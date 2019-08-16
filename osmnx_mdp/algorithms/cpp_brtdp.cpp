#include "cpp_brtdp.hpp"
#include <cfloat> // DBL_EPSILON
#include <stack>
#include <unordered_set>

#include "../serialize_util.hpp"
#include "../cpp_queue_util.hpp" // queue_decrease_priority, queue_pop


#include <chrono>


BRTDP::BRTDP() {};
BRTDP::~BRTDP() {};


// TODO: Same as in MDP
double BRTDP::get_Q_value(
        google::dense_hash_map<std::pair<long, long>, double, pair_hash> &v,
        const std::pair<long, long> &s_pair,
        const std::pair<long, long> &a)
{
    long s = s_pair.second;
    double future_cost = 0;

    for (const auto &outcome : (*this->P)[a]) {
        // TODO {s, outcome.first} is code duplication.
        future_cost += outcome.second * ((*this->C)[{s, outcome.first}] + v[{s, outcome.first}]);
    }

    // Penalize U-turns by increasing cost by 10%.
    // TODO: Is this fine ?
    if (a.second == s_pair.first) {
        future_cost *= 1.1;
    }

    return future_cost;
}

int BRTDP::init(
        std::vector<long> *S,
        google::dense_hash_map<long, std::vector<std::pair<long, long>>> *A,
        google::dense_hash_map<std::pair<long, long>, double, pair_hash> *C,
        google::dense_hash_map<
            std::pair<long, long>,
            std::vector<std::pair<long, double>>,
            pair_hash
        > *P,
        google::dense_hash_map<long, std::vector<long>> *predecessors,
        google::dense_hash_map<long, std::pair<double, double>> *data)
{
    this->S = S;
    this->A = A;
    this->C = C;
    this->P = P;

    this->predecessors = predecessors;
    this->data = data;

    this->vl.set_empty_key({0, 0});
    this->vu.set_empty_key({0, 0});

    return 0;
}

int BRTDP::setup(const long &start, const long &goal, std::unordered_map<std::string, double> cfg)
{
    this->start = start;
    this->goal = goal;

    this->alpha = cfg["alpha"];
    this->tau = cfg["tau"];

    std::default_random_engine rd;
    this->random_generator = std::default_random_engine(rd());

    const double lat2 = (*this->data)[this->goal].first;
    const double lon2 = (*this->data)[this->goal].second;

    for (const auto &x : *data) {
        const double lat1 = x.second.first;
        const double lon1 = x.second.second;

        for (auto &pred : (*this->predecessors)[x.first]) {
            this->vl[{pred, x.first}] = aerial_dist(lat1, lon1, lat2, lon2) / 200;
        }
    }

    this->init_upper_bound_heuristic();
    //this->init_lower_bound_heuristic();

    // TODO Move into function
    // TODO This assumes start has one predecessor..
    this->vl[{0, this->start}] = this->vl[{(*this->predecessors)[this->start][0], this->start}];
    this->vu[{0, this->start}] = this->vu[{(*this->predecessors)[this->start][0], this->start}];

    return 0;
}

std::pair<std::pair<long, long>, double> BRTDP::get_minimum_action(
        google::dense_hash_map<std::pair<long, long>, double, pair_hash> &v,
        const std::pair<long, long> &node_pair)
{
    std::pair<long, long> curr_min_action;
    double curr_min = INFINITY;
    for (const auto &a : (*this->A)[node_pair.second]) {
        double q_val = get_Q_value(v, node_pair, a);

        if (q_val < curr_min) {
            curr_min = q_val;
            curr_min_action = a;
        }
    }
    return std::pair<std::pair<long, long>, double>(curr_min_action, curr_min);
}

// TODO RENAME
std::pair<long, long> BRTDP::update_v(
        google::dense_hash_map<std::pair<long, long>, double, pair_hash> &v,
        const std::pair<long, long> &node_pair)
{
    const auto min_action = this->get_minimum_action(v, node_pair);
    v[node_pair] = min_action.second;
    return min_action.first;
}

int BRTDP::run_trials()
{
#ifdef TESTS
    save_brtdp(this, "BRTDPrun_trials.cereal");
    this->random_generator.seed(42069);
#endif
    std::pair<long, long> start_pair = {0, this->start};

    int i = 0;
    double last_diff = 0;

    while (this->vu[start_pair] - this->vl[start_pair] > this->alpha) {
        double diff = this->vu[start_pair] - this->vl[start_pair];

        if (diff < 1.0 && std::fabs(last_diff - diff) < DBL_EPSILON)
            break;

        last_diff = diff;

        this->run_trial(this->tau);

        i += 1;
    }
#ifdef TESTS
    save_brtdp(this, "BRTDPrun_trialsWANT.cereal");
#endif
    return i;
}

double BRTDP::get_outcome_distribution(
        const std::pair<long, long> &curr_min_action,
        std::vector<double> &distribution)
{
    // Generate a outcome distribution based on the inconsistency between the
    // upper bound and the lower bound, i.e. the more inconsistent, the higher
    // the chance to select that outcome.
    // BRTDP visits unexplored nodes that way.
    // TODO: There's also other ways to select the next node, check paper.
    double B = 0;
    for (auto &outcome : (*this->P)[curr_min_action]) {
        std::pair<long, long> node_pair = {curr_min_action.first, outcome.first};
        double cost = outcome.second * (this->vu[node_pair] - this->vl[node_pair]);
        distribution.push_back(cost);
        B += cost;
    }
    return B;
}

long BRTDP::select_node_probabilistically(
        const std::pair<long, long> &curr_min_action,
        const std::vector<double> &distribution_param)
{
    // Based on an action, select the next node out of the outcomes
    // probabilistcally.
    std::discrete_distribution<int> distribution(
        distribution_param.begin(),
        distribution_param.end()
    );
    const int n = distribution(this->random_generator);
    return (*this->P)[curr_min_action][n].first;
}

int BRTDP::run_trial(const double &tau)
{
    std::pair<long, long> start_pair = {0, this->start};
    std::pair<long, long> node_pair = start_pair;

    std::stack<std::pair<long, long>> traj;

    while (true) {
        if (node_pair.second == this->goal)
            break;

        traj.push(node_pair);

        this->update_v(this->vu, node_pair);
        const auto curr_min_action = this->update_v(this->vl, node_pair);

        std::vector<double> distribution_param;
        const double B = get_outcome_distribution(curr_min_action, distribution_param);

        if (B < ((this->vu[start_pair] - this->vl[start_pair]) / tau))
            break;

        node_pair = {
            node_pair.second,
            select_node_probabilistically(curr_min_action, distribution_param)
        };
    }

    while (!traj.empty()) {
        const std::pair<long, long> node_pair = traj.top();
        traj.pop();
        this->update_v(this->vu, node_pair);
        this->update_v(this->vl, node_pair);
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

    int failure_counter = 0;

    std::pair<long, long> curr_node_pair = {0, this->start};
    while (curr_node_pair.second != this->goal) {
        long curr_node = curr_node_pair.second;
        path.push_back(curr_node);

        // TODO: Remove this again.
        failure_counter += 1;
        if (failure_counter > 20000) {
            std::cout << " FAILURE. " << std::endl;
            break;
        }

        const long diverged_node = diverge_policy[curr_node];

        if (diverged_node == 0 || visited.find(diverged_node) != visited.end()) {
            const auto min_action = this->get_minimum_action(this->vu, curr_node_pair);
            const std::pair<long, long> last_node_pair = curr_node_pair;
            curr_node_pair = {last_node_pair.second, min_action.first.second};

            // TODO: This happens very rarely and only with very aggressive
            // diverge policies that are probably way too much.
            // Consider removing this.
            //
            // This is quite a difference to vanilla BRTDP. But it's needed,
            // else we crash because of our hard diverge policies.
            if (visited[curr_node_pair.second] > 5) {
                custom_updates += 1;
                // If we're looping, fix this by updating the value of the node.
                // This is most likely because we diverged too far and BRTDP
                // wasn't here before.
                // It takes a while for the costs to become big enough, i.e.
                // for the loop to resolve.
                this->update_v(this->vu, last_node_pair);
                curr_node_pair = {last_node_pair.second, this->update_v(this->vl, last_node_pair).second};
            }
        } else {
            curr_node_pair = {curr_node_pair.second, diverged_node};
        }

        visited[curr_node_pair.second] += 1;
    }

    // TODO Consider
    //path.push_back(this->goal);

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

void BRTDP::init_upper_bound_heuristic() {
    // Using DS-MPI as described in the paper.
    // TODO long ass function like wheres the self respect
    // TODO It works and I aint got time so let's postpone cleaning this.
    google::dense_hash_map<
        long,
        std::vector<std::pair<std::pair<long, long>, double>>
    > P_pred_lookup;

    P_pred_lookup.set_empty_key(0);

    for (auto &action_outcomes_pair : (*this->P)) {
        auto action = action_outcomes_pair.first;
        auto outcomes = action_outcomes_pair.second;

        for (auto &outcome : outcomes) {
            P_pred_lookup[outcome.first].push_back({action, outcome.second});
        }
    }

    // TODO: This has got to become a struct. lmao
    google::dense_hash_map<std::pair<long, long>, double, pair_hash> pg_hat;
    google::dense_hash_map<std::pair<long, long>, double, pair_hash> w_hat;
    google::dense_hash_map<long, double> w;
    google::dense_hash_map<long, double> pg;
    google::dense_hash_map<long, std::pair<long, long>> policy;
    google::dense_hash_map<long, std::pair<double, double>> priority;
    google::dense_hash_map<long, bool> fin;

    pg_hat.set_empty_key({0, 0});
    w_hat.set_empty_key({0, 0});
    pg.set_empty_key(0);
    w.set_empty_key(0);
    policy.set_empty_key(0);
    priority.set_empty_key(0);
    fin.set_empty_key(0);

    for (long &state : (*this->S)) {
        priority[state] = {INFINITY, INFINITY};
        fin[state] = false;
        for (auto &action : (*this->A)[state]) {
            pg_hat[action] = 0;
            w_hat[action] = (*this->C)[action];
        }
        pg[state] = 1;
        w[state] = 1;
    }

    for (auto &action : (*this->A)[this->goal]) {
        pg_hat[action] = 1;
    }

    // Paper: 'Select an action arbitrarily.'
    // So I just take the first action and noone can stop me.
    //policy[this->goal] = (*this->A)[this->goal][0];
    policy[this->goal] = {this->goal, this->goal};

    std::vector<std::pair<long, std::pair<double, double>>> queue;
    queue.push_back({this->goal, {0, 0}});

    while (!queue.empty()) {
        long x = queue_pop(queue);

        fin[x] = true;
        w[x] = w_hat[policy[x]];
        pg[x] = pg_hat[policy[x]];

        for (auto &action_chance_pair : P_pred_lookup[x]) {
            auto action = action_chance_pair.first;
            double chance = action_chance_pair.second;
            long y = action.first;

            if (fin[y])
                continue;

            w_hat[action] += chance * w[x];
            pg_hat[action] += chance * pg[x];

            std::pair<double, double> curr_priority = {1 - pg_hat[action], w_hat[action]};

            if (curr_priority < priority[y]) {
                priority[y] = curr_priority;
                policy[y] = action;

                queue_decrease_priority<long, std::pair<double, double>>(queue, y, curr_priority);
            }
        }
    }

    // Let's do it.. Theorem 3
    // Bigger than expected (that's what she said)
    google::dense_hash_map<std::pair<long, long>, double, pair_hash> lambda;

    lambda.set_empty_key({0, 0});

    for (auto &state_action_pair : (*this->A)) {
        auto actions = state_action_pair.second;
        for (auto &action : actions) {
            long x = action.first;

            auto outcomes = (*this->P)[action];

            double total_w = 0;
            double total_pg = 0;

            for (auto &outcome : outcomes) {
                long y = outcome.first;
                total_w += outcome.second * w[y];
                total_pg += outcome.second * pg[y];
            }

            if (pg[x] < total_pg - DBL_EPSILON) {
                lambda[action] = ((*this->C)[action] + total_w - w[x]) / (total_pg - pg[x]);
                // TODO: Below mention double imprecision ..
            } else if (w[x] + DBL_EPSILON >= (*this->C)[action] + total_w && std::fabs(pg[x] - total_pg) < DBL_EPSILON) {
                lambda[action] = 0;
            } else {
                lambda[action] = INFINITY;
            }
        }
    }

    // TODO: Too manual? The min_element alternative looks unreadable quite frankly.
    double max_lambda = -INFINITY;
    for (long &state : (*this->S)) {
        double min_action = INFINITY;

        for (auto action : (*this->A)[state]) {
            min_action = std::min(min_action, lambda[action]);
        }

        // TODO: In case there's a bug, uncomment.
        //if (min_action == INFINITY) {
        //    std::cout << " ----- INF "<< state << std::endl;
        //    min_action = 0;
        //}

        max_lambda = std::max(max_lambda, min_action);
    }

    for (long &state : (*this->S)) {
        for (long &neighbor : (*this->predecessors)[state]) {
            this->vu[{neighbor, state}] = w[state] + (1 - pg[state]) * max_lambda;
        }
    }
}

void BRTDP::init_lower_bound_heuristic() {
    // Single source (from goal) all target Dijkstra

    google::dense_hash_map<long, bool> fin;
    fin.set_empty_key(0);

    google::dense_hash_map<long, double> dist;
    google::dense_hash_map<long, long> prev;

    dist.set_empty_key(0);
    prev.set_empty_key(0);

    std::vector<std::pair<long, double>> queue;

    for (long &state : (*this->S)) {
        dist[state] = INFINITY;
    }
    dist[this->goal] = 0;
    queue.push_back({this->goal, 0});

    std::make_heap(queue.begin(), queue.end());

    auto starttime = std::chrono::high_resolution_clock::now();
    while (!queue.empty()) {
        long node = queue_pop(queue);

        fin[node] = true;

        for (long &neighbor : (*this->predecessors)[node]) {
            double new_dist = dist[node] + (*this->C)[{neighbor, node}];

            if (fin[neighbor])
                continue;

            // TODO
            //if (new_dist < dist[neighbor] + DBL_EPSILON) {
            if (new_dist < dist[neighbor]) {
                dist[neighbor] = new_dist;
                prev[neighbor] = node;

                queue_decrease_priority<long, double>(queue, neighbor, new_dist);
            }
        }
    }
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - starttime);
    std::cout << "Time taken by loop: " << duration.count() << " msec" << std::endl;


    for (long &state : (*this->S)) {
        double curr_cost = 0;
        long curr_node = state;

        while (curr_node != this->goal) {
            long next_node = prev[curr_node];
            curr_cost += (*this->C)[{curr_node, next_node}];
            curr_node = next_node;
        }

        for (long &neighbor : (*this->predecessors)[state]) {
            //std::cout << this->vl[{neighbor, state}] << " >> " << curr_cost << std::endl;
            this->vl[{neighbor, state}] = curr_cost;
        }
    }
}
