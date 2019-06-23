#include "cpp_brtdp.hpp"
#include <stack>
#include <unordered_set>

#include "../serialize_util.hpp"


BRTDP::BRTDP() {};
BRTDP::~BRTDP() {};

double BRTDP::get_Q_value(
        google::dense_hash_map<long, double> &v,
        const long &s,
        const std::pair<long, long> &a)
{
    const double immediate_cost = (*this->C)[a];
    double future_cost = 0;
    for (const auto &outcome : (*this->P)[s][a]) {
        future_cost += outcome.second * v[outcome.first];
    }
    return immediate_cost + future_cost;
}

int BRTDP::init(
        std::vector<long> *S,
        google::dense_hash_map<long, std::vector<std::pair<long, long>>> *A,
        google::dense_hash_map<std::pair<long, long>, float, pair_hash> *C,
        google::dense_hash_map<
            long,
            google::dense_hash_map<
                std::pair<long, long>,
                std::vector<std::pair<long, double>>,
                pair_hash
            >
        > *P,
        google::dense_hash_map<long, std::pair<float, float>> *data)
{
    this->S = S;
    this->A = A;
    this->C = C;
    this->P = P;

    this->data = data;

    this->vl.set_empty_key(0);
    this->vu.set_empty_key(0);

    return 0;
}

int BRTDP::setup(const long &start, const long &goal)
{
    this->start = start;
    this->goal = goal;

    std::default_random_engine rd;
    this->random_generator = std::default_random_engine(rd());

    const double lat2 = (*this->data)[this->goal].first;
    const double lon2 = (*this->data)[this->goal].second;

    for (const auto &x : *data) {
        const double lat1 = x.second.first;
        const double lon1 = x.second.second;

        // TODO is this best choice ? 50 and 10 ?
        this->vl[x.first] = aerial_dist(lat1, lon1, lat2, lon2) / 160;
        this->vu[x.first] = aerial_dist(lat1, lon1, lat2, lon2) / 10;
    }

    return 0;
}

std::pair<std::pair<long, long>, double> BRTDP::get_minimum_action(
        google::dense_hash_map<long, double> &v,
        const long &node)
{
    std::pair<long, long> curr_min_action;
    double curr_min = INFINITY;
    for (const auto &a : (*this->A)[node]) {
        double q_val = get_Q_value(v, node, a);
        if (q_val < curr_min) {
            curr_min = q_val;
            curr_min_action = a;
        }
    }
    return std::pair<std::pair<long, long>, double>(curr_min_action, curr_min);
}

// TODO RENAME
std::pair<long, long> BRTDP::update_v(
        google::dense_hash_map<long, double> &v,
        const long &node)
{
    const auto min_action = this->get_minimum_action(v, node);
    v[node] = min_action.second;
    return min_action.first;
}

int BRTDP::run_trials(const double &alpha, const double &tau)
{
    // Defaults:
    //  alpha = 1e-10
    //  t = 10
#ifdef TESTS
    save_brtdp(this, "BRTDPrun_trials.cereal");
    this->random_generator.seed(42069);
#endif
    int i = 0;
    double last_diff = 0;
    while (this->vu[this->start] - this->vl[this->start] > alpha) {
        double diff  = this->vu[this->start] - this->vl[this->start];
        if (last_diff == diff) {
            std::cout << last_diff << std::endl;
            break;
        }
        last_diff = diff;

        this->run_trial(tau);
        i += 1;
    }
#ifdef TESTS
    save_brtdp(this, "BRTDPrun_trialsWANT.cereal");
#endif
    return i;
}

double BRTDP::get_outcome_distribution(
        const std::pair<long, long> &curr_min_action,
        std::vector<float> &distribution)
{
    // Generate a outcome distribution based on the inconsistency between the
    // upper bound and the lower bound, i.e. the more inconsistent, the higher
    // the chance to select that outcome.
    // BRTDP visits unexplored nodes that way.
    // TODO: There's also other ways to select the next node, check paper.
    const long node = curr_min_action.first;
    double B = 0;
    for (auto &outcome : (*this->P)[node][curr_min_action]) {
        double cost = outcome.second * (this->vu[outcome.first] - this->vl[outcome.first]);
        distribution.push_back(cost);
        B += cost;
    }
    return B;
}

long BRTDP::select_node_probabilistically(
        const std::pair<long, long> &curr_min_action,
        const std::vector<float> &distribution_param)
{
    // Based on an action, select the next node out of the outcomes
    // probabilistcally.
    std::discrete_distribution<int> distribution(distribution_param.begin(), distribution_param.end());
    const int n = distribution(this->random_generator);
    return (*this->P)[curr_min_action.first][curr_min_action][n].first;
}

int BRTDP::run_trial(const double &tau)
{
    long node = this->start;

    std::stack<long> traj;

    while (true) {
        if (node == this->goal)
            break;

        traj.push(node);

        this->update_v(this->vu, node);
        const auto curr_min_action = this->update_v(this->vl, node);

        std::vector<float> distribution_param;
        const double B = get_outcome_distribution(curr_min_action, distribution_param);

        if (B < ((this->vu[this->start] - this->vl[this->start]) / tau))
            break;

        node = select_node_probabilistically(curr_min_action, distribution_param);
    }

    while (!traj.empty()) {
        const long node = traj.top();
        traj.pop();
        this->update_v(this->vu, node);
        this->update_v(this->vl, node);
    }

    return 0;
}

std::vector<long> BRTDP::get_path(
        google::dense_hash_map<long, long> &diverge_policy)
{
#ifdef TESTS
    save_brtdp(this, "BRTDPget_path.cereal");
#endif
    std::vector<long> path;

    google::dense_hash_map<long, int> visited;
    visited.set_empty_key(0);
    visited[this->start] = 1;

    int custom_updates = 0;

    long curr_node = this->start;
    while (curr_node != this->goal) {
        path.push_back(curr_node);

        const long diverged_node = diverge_policy[curr_node];
        if (diverged_node == 0 || visited.find(diverged_node) != visited.end()) {
            const auto min_action = this->get_minimum_action(this->vu, curr_node);
            const long last_node = curr_node;
            curr_node = min_action.first.second;

            // This is quite a difference to vanilla BRTDP. But it's needed,
            // else we crash because of our hard diverge policies.
            if (visited[curr_node] > 5) {
                custom_updates += 1;
                // If we're looping, fix this by updating the value of the node.
                // This is most likely because we diverged too far and BRTDP
                // wasn't here before.
                // It takes a while for the costs to become big enough, i.e.
                // for the loop to resolve.
                curr_node = this->update_v(this->vl, last_node).second;
            }
        } else {
            curr_node = diverged_node;
        }

        visited[curr_node] += 1;
    }

    if (custom_updates > 0) {
        std::cout << "custom_updates "  << custom_updates << std::endl;
    }

#ifdef TESTS
    {
        std::ofstream os("BRTDPget_pathWANT.cereal", std::ios::binary);
        cereal::BinaryOutputArchive archive(os);
        archive(path);
    }
#endif

    return path;
}
