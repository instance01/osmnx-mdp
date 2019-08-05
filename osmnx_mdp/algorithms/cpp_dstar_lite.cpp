#include <cmath> // sin, asin, cos, pow, M_PI
#include <cfloat> // DBL_EPSILON
#include <algorithm> // min_element
#include <stdexcept> // runtime_error
#include <unordered_set>

#include "cpp_dstar_lite.hpp"

#include "../serialize_util.hpp"
#include "../cpp_queue_util.hpp" // queue_decrease_priority, queue_pop


DStar_Lite::DStar_Lite () {}
DStar_Lite::~DStar_Lite () {}

int DStar_Lite::init(
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

    this->rhs.set_empty_key(0);
    this->g.set_empty_key(0);
    this->U.set_empty_key(0);
    this->U.set_deleted_key(1);

    return 0;
}

int DStar_Lite::setup(const long &start, const long &goal, std::unordered_map<std::string, double> cfg)
{
    this->start = start;
    this->goal = goal;

    this->init_heuristic();

    // TODO: Explanations
    // rhs, g:
    // Estimate of distance from current node to start node
    // rhs is an estimate using g value of predecessors + distance from
    // current node to the predecessor.
    // g is the previously calculated g-value (similar to A*).
    // Thus rhs will mostly be a bit ahead of g.

    // TODO Per paper: It's not necessary to init all states to inf here
    // We can also do it when we encounter a new state in
    // compute_shortest_path
    for (const auto &node : this->nodes) {
        this->rhs[node] = INFINITY;
        this->g[node] = INFINITY;
    }

    this->rhs[this->goal] = 0;
    this->U[this->goal] = this->calculate_key(this->goal);

    return 0;
}

std::pair<long, double> DStar_Lite::get_min_successor(const long &node)
{
    // For a given node, return the successor which lies on the minimum sum
    // of the estimated path to the node and the edge cost between the node
    // and the successor.

    double min_cost = INFINITY;
    long min_node = 0;

    // Manually find minimum. An alternative would be to use min_element,
    // however it is actually less readable in this case.
    for (const auto &succ : (*this->successors)[node]) {
        const double edge_cost = (*this->cost)[{node, succ}];
        const double cost = this->g[succ] + edge_cost;

        if (cost <= min_cost) {
            min_cost = cost;
            min_node = succ;
        }
    }

    return {min_node, min_cost};
}

// TODO Rename DStar_Lite

std::pair<double, double> DStar_Lite::calculate_key(const long &node)
{
    double key = std::min(this->g[node], this->rhs[node]);
    return std::pair<double, double>(key + this->heuristic_map[node] + this->k, key);
}

int DStar_Lite::update_vertex(const long &node)
{
    if (node != this->goal)
        this->rhs[node] = this->get_min_successor(node).second;

    this->U.erase(node);

    if (this->g[node] != this->rhs[node])
        this->U[node] = this->calculate_key(node);

    return 0;
}

int DStar_Lite::compute_shortest_path()
{
#ifdef TESTS
    save_dstar(this, "DSTARcompute_shortest_path.cereal");
#endif
    int i = 0;
    while (!this->U.empty()) {
        i += 1;
        const auto candidate = *std::min_element(
                this->U.begin(),
                this->U.end(),
                [](auto& a, auto& b) { return a.second < b.second; });

        const bool reached_start = candidate.second >= this->calculate_key(this->start);
        const bool start_consistent = this->rhs[this->start] == this->g[this->start];
        if (reached_start && start_consistent)
            break;

        const long node = candidate.first;
        const auto k_old = candidate.second;

        this->U.erase(node);
        const auto key = this->calculate_key(node);

        if (k_old < key) {
            this->U[node] = key;
        } else if (this->g[node] > this->rhs[node]) {
            this->g[node] = this->rhs[node];
        } else {
            this->g[node] = INFINITY;
            this->update_vertex(node);
        }

        for (const auto &pred : (*this->predecessors)[node]) {
            this->update_vertex(pred);
        }
    }
    
#ifdef TESTS
    save_dstar(this, "DSTARcompute_shortest_pathWANT.cereal");
#endif

    return i;
}

int DStar_Lite::drive(
        std::vector<long> &out,
        google::dense_hash_map<long, long> &diverge_policy)
{
#ifdef TESTS
    save_dstar(this, "DSTARdrive.cereal");
#endif
    long last_start = this->start;

    std::unordered_set<long> visited = {this->start};
    out.push_back(this->start);

    while (this->start != this->goal) {
        if (this->g[this->start] == INFINITY)
            throw std::runtime_error("No path found.");

        const long diverged_node = diverge_policy[this->start];
        if (diverged_node == 0 || visited.find(diverged_node) != visited.end()) {
            this->start = this->get_min_successor(this->start).first;
        } else {
            this->start = diverged_node;
            this->k += this->heuristic_map[last_start];
            last_start = this->start;
            this->compute_shortest_path();
        }

        visited.insert(this->start);
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

// TODO: COPIED (99%) FROM BRTDP
void DStar_Lite::init_heuristic() {
    // Single source (from goal) all target Dijkstra

    this->heuristic_map.set_empty_key(0);

    google::dense_hash_map<long, double> dist;
    google::dense_hash_map<long, long> prev;

    dist.set_empty_key(0);
    prev.set_empty_key(0);

    std::vector<std::pair<long, double>> queue;

    for (long &state : this->nodes) {
        dist[state] = INFINITY;
        queue.push_back({state, INFINITY});
    }
    dist[this->goal] = 0;

    std::make_heap(queue.begin(), queue.end());

    while (!queue.empty()) {
        long node = queue_pop(queue);

        for (long &neighbor : (*this->predecessors)[node]) {
            double new_dist = dist[node] + (*this->cost)[{neighbor, node}];

            if (new_dist < dist[neighbor] + DBL_EPSILON) {
                dist[neighbor] = new_dist;
                prev[neighbor] = node;

                queue_decrease_priority<long, double>(queue, neighbor, new_dist);
            }
        }
    }

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
