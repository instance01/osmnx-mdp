#include <cmath> // sin, asin, cos, pow, M_PI
#include <cfloat> // DBL_EPSILON
#include <algorithm> // min_element
#include <stdexcept> // runtime_error
#include <unordered_set>

#include "cpp_dstar_lite.hpp"

#include "../serialize_util.hpp"
#include "../cpp_queue_util.hpp" // queue_decrease_priority, queue_pop


DStarLite::DStarLite () {}
DStarLite::~DStarLite () {}


int DStarLite::init(
        google::dense_hash_map<long, std::vector<long>> *predecessors,
        google::dense_hash_map<long, std::vector<long>> *successors,
        google::dense_hash_map<std::pair<long, long>, double, pair_hash> *cost,
        google::dense_hash_map<long, std::pair<double, double>> *data)
{
    for (auto &x : *successors) {
        this->nodes.push_back(x.first);
    }
    this->cost = cost;
    this->data = data;
    this->predecessors = predecessors;
    this->successors = successors;

    this->rhs.set_empty_key({0, 0});
    this->g.set_empty_key({0, 0});
    this->U.set_empty_key({0, 0});
    this->U.set_deleted_key({1, 1});

    return 0;
}

int DStarLite::setup(const long &start, const long &goal, std::unordered_map<std::string, double> cfg)
{
    this->start = start;
    this->goal = goal;

    this->k = 0;

    if (cfg["heuristic"] == 0) {
        this->dijkstra_heuristic = true;
        this->init_heuristic();
    } else {
        this->heuristic_max_speed = cfg["heuristic_max_speed"];
        this->dijkstra_heuristic = false;
    }

    (*this->predecessors)[this->start].push_back(0);

    // TODO Per paper: It's not necessary to init all states to inf here
    // We can also do it when we encounter a new state in
    // compute_shortest_path
    for (const auto &node : this->nodes) {
        for (const auto &pred : (*this->predecessors)[node]) {
            this->rhs[{pred, node}] = INFINITY;
            this->g[{pred, node}] = INFINITY;
        }
    }

    for (const auto &pred : (*this->predecessors)[this->goal]) {
        this->rhs[{pred, this->goal}] = 0;
        this->U[{pred, this->goal}] = this->calculate_key({pred, this->goal});
    }

    this->curr_start = {0, this->start};

    return 0;
}

std::pair<long, double> DStarLite::get_min_successor(const std::pair<long, long> &node_pair)
{
    // For a given node, return the successor which lies on the minimum sum of
    // the estimated path to the node g(x) and the edge cost between the node
    // and the successor.
    const long node = node_pair.second;

    double min_cost = INFINITY;
    long min_node = 0;

    // Manually find minimum. An alternative would be to use min_element,
    // however it is actually less readable in this case.
    for (const auto &succ : (*this->successors)[node]) {
        const double edge_cost = (*this->cost)[{node, succ}];
        double cost = this->g[{node, succ}] + edge_cost;

        // Penalize U-turns.
        if (node_pair.first == succ) {
            cost += U_TURN_PENALTY;
        }

        if (cost <= min_cost) {
            min_cost = cost;
            min_node = succ;
        }
    }

    return {min_node, min_cost};
}

float DStarLite::aerial_heuristic(const long &node)
{
    float lat1, lon1, lat2, lon2;
    std::tie(lat1, lon1) = (*this->data)[node];
    std::tie(lat2, lon2) = (*this->data)[this->start];
    return aerial_dist(lat1, lon1, lat2, lon2) / this->heuristic_max_speed; // Hours
}

std::pair<double, double> DStarLite::calculate_key(const std::pair<long, long> &node_pair)
{
    double key = std::min(this->g[node_pair], this->rhs[node_pair]);
    double heuristic_val = this->dijkstra_heuristic ? this->heuristic_map[node_pair.second] : aerial_heuristic(node_pair.second);
    return std::pair<double, double>(key + heuristic_val + this->k, key);
}

int DStarLite::update_vertex(const std::pair<long, long> &node_pair)
{
    if (this->g[node_pair] != this->rhs[node_pair])
        this->U[node_pair] = this->calculate_key(node_pair);
    else if(this->g[node_pair] == this->rhs[node_pair])
        this->U.erase(node_pair);

    return 0;
}

int DStarLite::compute_shortest_path()
{
#ifdef TESTS
    save_dstar(this, "DSTARcompute_shortest_path.cereal");
#endif

    // TODO Horrendous code quality.
    // TODO Horrendous code quality.
    // TODO Horrendous code quality.


    int i = 0;
    while (!this->U.empty()) {
        i += 1;

        const auto candidate = *std::min_element(
                this->U.begin(),
                this->U.end(),
                [](auto& a, auto& b) { return a.second < b.second; });

        const bool reached_start = candidate.second >= this->calculate_key(this->curr_start);
        const bool start_consistent = this->rhs[this->curr_start] == this->g[this->curr_start];
        if (reached_start && start_consistent)
            break;
        // TODO: Different from the optimized version of D* Lite
        //const bool start_not_overconsistent = this->rhs[this->start] <= this->g[this->start];
        //if (reached_start && start_not_overconsistent)
        //    break;

        const std::pair<long, long> node_pair = candidate.first;
        const auto k_old = candidate.second;

        const auto key = this->calculate_key(node_pair);

        if (k_old < key) {
            this->U[node_pair] = key;
        } else if (this->g[node_pair] > this->rhs[node_pair]) {
            this->g[node_pair] = this->rhs[node_pair];
            this->U.erase(node_pair);
            for (const auto &pred : (*this->predecessors)[node_pair.second]) {
                for (const auto &predpred : (*this->predecessors)[pred]) {
                    double penalty = predpred == node_pair.second ? U_TURN_PENALTY : 0.0;

                    if (pred != this->goal)
                        this->rhs[{predpred, pred}] = std::min(
                                this->rhs[{predpred, pred}],
                                ((*this->cost)[{pred, node_pair.second}] + this->g[node_pair]) + penalty);
                    this->update_vertex({predpred, pred});
                }
            }
        } else {
            const auto g_old = this->g[node_pair];
            this->g[node_pair] = INFINITY;

            // TODO: Pushing back might potentially rather SLOW it down than make it faster..
            auto to_update = (*this->predecessors)[node_pair.second];
            to_update.push_back(node_pair.second);
            for (const auto &x : to_update) {
                for (const auto &pred : (*this->predecessors)[x]) {
                    if (this->rhs[{pred, x}] == (*this->cost)[{x, node_pair.second}] + g_old) {
                        if (x != this->goal) {
                            this->rhs[{pred, x}] = this->get_min_successor({pred, x}).second;
                        }
                    }
                    this->update_vertex({pred, x});
                }
            }
        }
    }
    
#ifdef TESTS
    save_dstar(this, "DSTARcompute_shortest_pathWANT.cereal");
#endif

    return i;
}

int DStarLite::drive(
        std::vector<long> &out,
        google::dense_hash_map<long, long> &diverge_policy)
{
#ifdef TESTS
    save_dstar(this, "DSTARdrive.cereal");
#endif
    std::pair<long, long> start_pair = {0, this->start};
    long last_start = this->start;

    std::unordered_set<std::pair<long, long>, pair_hash> visited = {start_pair};
    out.push_back(this->start);

    while (this->start != this->goal) {
        if (this->g[{0, this->start}] == INFINITY)
            throw std::runtime_error("No path found.");

        const long diverged_node = diverge_policy[this->start];

        const std::pair<long, long> diverged_pair = {start_pair.second, diverged_node};
        if (diverged_node == 0 || visited.find(diverged_pair) != visited.end()) {
            this->start = this->get_min_successor(start_pair).first;
        } else {
            this->start = diverged_node;
            double heuristic_val = this->dijkstra_heuristic ?
                this->heuristic_map[last_start] : aerial_heuristic(last_start);
            this->k += heuristic_val;
            last_start = this->start;
            this->curr_start = diverged_pair;
            this->compute_shortest_path();
        }

        start_pair = {start_pair.second, this->start};

        visited.insert(start_pair);
        out.push_back(this->start);
    }

#ifdef TESTS
    {
        std::ofstream os("DSTARdriveWANT.cereal", std::ios::binary);
        cereal::BinaryOutputArchive archive(os);
        archive(out);
    }
#endif

    return 0;
}

void DStarLite::init_heuristic() {
    // Single source (from goal) all target Dijkstra
    auto prev = dijkstra(&this->nodes, this->cost, this->predecessors, this->goal);

    for (long &state : this->nodes) {
        double curr_cost = 0;
        long curr_node = state;

        while (curr_node != this->goal) {
            long next_node = prev[curr_node];
            curr_cost += (*this->cost)[{curr_node, next_node}];
            curr_node = next_node;
        }

        this->heuristic_map[state] = curr_cost;
    }
}
