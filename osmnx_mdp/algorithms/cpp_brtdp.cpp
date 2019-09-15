#include "cpp_brtdp.hpp"
#include <cfloat> // DBL_EPSILON
#include <stack>
#include <unordered_set>
#include "../cpp_queue_util.hpp" // queue_decrease_priority, queue_pop

#ifdef TESTS
#include "../serialize_util.hpp"
#else
#include <iostream>
#endif

#include <chrono>


BRTDP::BRTDP() {};
BRTDP::~BRTDP() {};


double BRTDP::get_Q_value(
        google::dense_hash_map<std::pair<long, long>, double, pair_hash> &v,
        const std::pair<long, long> &s_pair,
        const std::pair<long, long> &a)
{
    long s = s_pair.second;
    double future_cost = 0;

    for (const auto &outcome : (*this->P)[s_pair.first][a]) {
        std::pair<long, long> node_pair = {s, outcome.first};
        future_cost += outcome.second * ((*this->C)[node_pair] + v[node_pair]);
    }

    if (a.second == s_pair.first) {
        future_cost += U_TURN_PENALTY;
    }

    return future_cost;
}

int BRTDP::init(
        std::vector<long> *S,
        google::dense_hash_map<long, std::vector<std::pair<long, long>>> *A,
        google::dense_hash_map<std::pair<long, long>, double, pair_hash> *C,
        google::dense_hash_map<
            long,
            google::dense_hash_map<
                std::pair<long, long>,
                std::vector<std::pair<long, double>>,
                pair_hash
            >
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

    if (cfg["heuristic"] == 0) {
        this->init_lower_bound_heuristic();
    } else {
        this->heuristic_max_speed = cfg["heuristic_max_speed"];

        const double lat2 = (*this->data)[this->goal].first;
        const double lon2 = (*this->data)[this->goal].second;

        for (const auto &x : *data) {
            const double lat1 = x.second.first;
            const double lon1 = x.second.second;

            for (auto &pred : (*this->predecessors)[x.first]) {
                double dist = aerial_dist(lat1, lon1, lat2, lon2);
                this->vl[{pred, x.first}] = dist / this->heuristic_max_speed;
            }
        }
    }

    this->init_upper_bound_heuristic();

    // TODO Move into function
    // TODO This assumes start has one predecessor..
    this->vl[{1, this->start}] = this->vl[{(*this->predecessors)[this->start][0], this->start}];
    this->vu[{1, this->start}] = this->vu[{(*this->predecessors)[this->start][0], this->start}];

    return 0;
}

std::pair<std::pair<long, long>, double> BRTDP::get_minimum_action(
        google::dense_hash_map<std::pair<long, long>, double, pair_hash> &v,
        const std::pair<long, long> &node_pair)
{
    std::pair<long, long> curr_min_action;
    double curr_min = INFINITY;
    for (const auto &a : (*this->A)[node_pair.second]) {
        double q_val = this->get_Q_value(v, node_pair, a);

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
    std::pair<long, long> start_pair = {1, this->start};

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
        const long pred,
        const std::pair<long, long> &curr_min_action,
        std::vector<double> &distribution)
{
    // Generate a outcome distribution based on the inconsistency between the
    // upper bound and the lower bound, i.e. the more inconsistent, the higher
    // the chance to select that outcome.
    // BRTDP visits unexplored nodes that way.
    // TODO: There's also other ways to select the next node, check paper.
    double B = 0;
    for (auto &outcome : (*this->P)[pred][curr_min_action]) {
        std::pair<long, long> node_pair = {curr_min_action.first, outcome.first};
        double cost = outcome.second * (this->vu[node_pair] - this->vl[node_pair]);
        distribution.push_back(cost);
        B += cost;
    }
    return B;
}

long BRTDP::select_node_probabilistically(
        const long pred,
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
    return (*this->P)[pred][curr_min_action][n].first;
}

int BRTDP::run_trial(const double &tau)
{
    std::pair<long, long> start_pair = {1, this->start};
    std::pair<long, long> node_pair = start_pair;

    std::stack<std::pair<long, long>> traj;

    while (true) {
        if (node_pair.second == this->goal)
            break;

        traj.push(node_pair);

        this->update_v(this->vu, node_pair);
        const auto curr_min_action = this->update_v(this->vl, node_pair);

        std::vector<double> distribution_param;
        const double B = get_outcome_distribution(
                node_pair.first,
                curr_min_action,
                distribution_param);

        if (B < ((this->vu[start_pair] - this->vl[start_pair]) / tau))
            break;

        node_pair = {
            node_pair.second,
            select_node_probabilistically(
                    node_pair.first,
                    curr_min_action,
                    distribution_param)
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

    std::pair<long, long> curr_node_pair = {1, this->start};
    while (curr_node_pair.second != this->goal) {
        long curr_node = curr_node_pair.second;
        path.push_back(curr_node);

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
    // TODO Extremely long function, break up without losing expressiveness of
    // the algorithm
    google::dense_hash_map<
        long,
        std::vector<std::pair<std::tuple<long, long, long>, double>>
    > P_pred_lookup;

    P_pred_lookup.set_empty_key(0);

    for (auto &pred_rest_pair : (*this->P)) {
        auto pred = pred_rest_pair.first;

        for (auto &action_outcomes_pair : (*this->P)[pred]) {
            auto action = action_outcomes_pair.first;
            auto outcomes = action_outcomes_pair.second;

            for (auto &outcome : outcomes) {
                P_pred_lookup[outcome.first].push_back(
                    {
                        {pred, action.first, action.second},
                        outcome.second
                    }
                );
            }
        }
    }

    // TODO: This has got to become a struct, i.e. map pair to struct.
    // TODO: Hashmaps of hashmaps sucks.
    google::dense_hash_map<
        long,
        google::dense_hash_map<std::pair<long, long>, double, pair_hash>
    > pg_hat;
    google::dense_hash_map<
        long,
        google::dense_hash_map<std::pair<long, long>, double, pair_hash>
    > w_hat;
    google::dense_hash_map<std::pair<long, long>, double, pair_hash> w;
    google::dense_hash_map<std::pair<long, long>, double, pair_hash> pg;
    google::dense_hash_map<std::pair<long, long>, std::pair<long, long>, pair_hash> policy;
    google::dense_hash_map<std::pair<long, long>, std::pair<double, double>, pair_hash> priority;
    google::dense_hash_map<std::pair<long, long>, bool, pair_hash> fin;

    pg_hat.set_empty_key(0);
    w_hat.set_empty_key(0);
    pg.set_empty_key({0, 0});
    w.set_empty_key({0, 0});
    policy.set_empty_key({0, 0});
    priority.set_empty_key({0, 0});
    fin.set_empty_key({0, 0});

    for (long &state : (*this->S)) {
        pg_hat[state].set_empty_key({0, 0});
        w_hat[state].set_empty_key({0, 0});
    }
    pg_hat[1].set_empty_key({0, 0});
    w_hat[1].set_empty_key({0, 0});

    for (long &state : (*this->S)) {
        for (long &pred : (*this->predecessors)[state]) {
            std::pair<long, long> node_pair(pred, state);
            priority[node_pair] = {INFINITY, INFINITY};
            fin[node_pair] = false;

            for (auto &action : (*this->A)[state]) {
                pg_hat[pred][action] = 0;
                w_hat[pred][action] = (*this->C)[action];
            }
            pg[node_pair] = 1;
            w[node_pair] = 1;
        }
    }

    for (auto &action : (*this->A)[this->goal]) {
        for (long &pred : (*this->predecessors)[this->goal]) {
            pg_hat[pred][action] = 1;
        }
    }
    pg_hat[this->goal][{this->goal, this->goal}] = 1;

    for (long &pred : (*this->predecessors)[this->goal]) {
        policy[{pred, this->goal}] = {this->goal, this->goal};
    }
    policy[{this->goal, this->goal}] = {this->goal, this->goal};

    std::vector<
        std::pair<
            std::pair<long, long>,
            std::pair<double, double>
        >
    > queue;

    queue.push_back({{(*this->predecessors)[this->goal][0], this->goal}, {0, 0}});

    while (!queue.empty()) {
        auto x = queue_pop(queue);

        fin[x] = true;
        w[x] = w_hat[x.first][policy[x]];
        pg[x] = pg_hat[x.first][policy[x]];

        for (auto &action_chance_pair : P_pred_lookup[x.second]) {
            long pred, action_first, action_second;
            std::tie(pred, action_first, action_second) = action_chance_pair.first;
            std::pair<long, long> action(action_first, action_second);
            double chance = action_chance_pair.second;
            std::pair<long, long> y = {pred, action.first};

            if (x.first != action.first)
                continue;

            if (fin[y])
                continue;

            w_hat[pred][action] += chance * w[x];
            pg_hat[pred][action] += chance * pg[x];

            std::pair<double, double> curr_priority = {1 - pg_hat[pred][action], w_hat[pred][action]};

            if (curr_priority < priority[y]) {
                priority[y] = curr_priority;
                policy[y] = action;

                queue_decrease_priority<
                    std::pair<long, long>,
                    std::pair<double, double>
                >(queue, y, curr_priority);
            }
        }
    }

    // Done with the DS-MPI sweep.
    // Now, apply Theorem 3 from the BRTDP paper.
    this->apply_theorem_three(w, pg);
}

void BRTDP::apply_theorem_three(
        google::dense_hash_map<std::pair<long, long>, double, pair_hash> w,
        google::dense_hash_map<std::pair<long, long>, double, pair_hash> pg)
{
    // Bigger than expected (that's what she said)
    google::dense_hash_map<std::pair<long, long>, double, pair_hash> lambda;

    lambda.set_empty_key({0, 0});

    for (auto &state_action_pair : (*this->A)) {
        auto actions = state_action_pair.second;
        for (auto &action : actions) {
            long x = action.first;

            for (const long &pred : (*this->predecessors)[x]) {
                auto outcomes = (*this->P)[pred][action];

                double total_w = 0;
                double total_pg = 0;

                for (auto &outcome : outcomes) {
                    long y = outcome.first;
                    total_w += outcome.second * w[{x, y}];
                    total_pg += outcome.second * pg[{x, y}];
                }

                if (pg[{pred, x}] < total_pg - DBL_EPSILON) {
                    lambda[action] = ((*this->C)[action] + total_w - w[{pred, x}]) / (total_pg - pg[{pred, x}]);
                    // TODO: Below mention double imprecision ..
                } else if (w[{pred, x}] + DBL_EPSILON >= (*this->C)[action] + total_w && std::fabs(pg[{pred, x}] - total_pg) < DBL_EPSILON) {
                    lambda[action] = 0;
                } else {
                    lambda[action] = INFINITY;
                }
            }
        }
    }

    lambda[{this->goal, this->goal}] = 0;

    double max_lambda = -INFINITY;
    for (long &state : (*this->S)) {
        double min_action = INFINITY;

        for (auto action : (*this->A)[state]) {
            min_action = std::min(min_action, lambda[action]);
        }

        // This is not as described in the paper. It only happens with huge maps
        // such as Bavaria.
        // Possible ideas could be that dead end removal is not exhaustive yet
        // (e.g. dead ends that look in more than 2 nodes) or there is a bug
        // somewhere else in the DS-MPI implementation.
        if (min_action == INFINITY) {
            std::cout << " --- INF "<< state << std::endl;
            min_action = 0;
        }

        max_lambda = std::max(max_lambda, min_action);
    }

    std::cout << "Max lambda: " << max_lambda << std::endl;

    // Finally, the upper bound is initialized.
    for (long &state : (*this->S)) {
        for (long &neighbor : (*this->predecessors)[state]) {
            std::pair<long, long> node_pair = {neighbor, state};
            this->vu[node_pair] = w[node_pair] + (1 - pg[node_pair]) * max_lambda;
        }
    }
}

void BRTDP::init_lower_bound_heuristic() {
    // Single source (from goal) all target Dijkstra
    auto prev = dijkstra(this->S, this->C, this->predecessors, this->goal);

    for (long &state : (*this->S)) {
        double curr_cost = 0;
        long curr_node = state;

        while (curr_node != this->goal) {
            long next_node = prev[curr_node];
            curr_cost += (*this->C)[{curr_node, next_node}];
            curr_node = next_node;
        }

        for (long &neighbor : (*this->predecessors)[state]) {
            this->vl[{neighbor, state}] = curr_cost;
        }
    }
}
