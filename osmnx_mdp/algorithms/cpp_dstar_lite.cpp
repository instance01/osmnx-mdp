#include <cmath> // sin, asin, cos, pow, M_PI
#include <algorithm> // min_element
#include <stdexcept> // runtime_error
#include <unordered_set>

#include "cpp_dstar_lite.hpp"

// TODO Explanations.
/*
rhs, g:
Estimate of distance from current node to start node
rhs is an estimate using g value of predecessors + distance from
current node to the predecessor.
g is the previously calculated g-value (similar to A*).
Thus rhs will mostly be a bit ahead of g.
*/

// TODO For U consider using some kind of queue?

cpp_DStar_Lite::cpp_DStar_Lite () { }

cpp_DStar_Lite::~cpp_DStar_Lite () { }

int cpp_DStar_Lite::init(
        google::dense_hash_map<long, std::vector<long>> *predecessors,
        google::dense_hash_map<long, std::vector<long>> *successors,
        google::dense_hash_map<std::pair<long, long>, float, pair_hash> *cost,
        google::dense_hash_map<long, std::pair<float, float>> *data) {
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

int cpp_DStar_Lite::setup(long start, long goal) {
    this->start = start;
    this->goal = goal;

    // TODO Per paper: It's not necessary to init all states to inf here
    // We can also do it when we encounter a new state in
    // compute_shortest_path
    for (auto &node : this->nodes) {
        this->rhs[node] = INFINITY;
        this->g[node] = INFINITY;
    }

    this->rhs[this->goal] = 0;
    this->U[this->goal] = this->calculate_key(this->goal);

    return 0;
}

std::pair<long, float> cpp_DStar_Lite::get_min_successor(long node) {
    float min_cost = INFINITY;
    long min_node = 0;

    // Manually find minimum. An alternative would be to use min_element,
    // however it is actually less readable in this case.
    for (auto &succ : (*this->successors)[node]) {
        float edge_cost = (*this->cost)[std::pair<long, long>(node, succ)];
        float cost = this->g[succ] + edge_cost;

        if (cost <= min_cost) {
            min_cost = cost;
            min_node = succ;
        }
    }

    return std::pair<long, float> (min_node, min_cost);
}

// TODO Rename cpp_DStar_Lite
float cpp_DStar_Lite::heuristic(long node) {
    float lat1, lon1, lat2, lon2;
    std::tie(lat1, lon1) = (*this->data)[node];
    std::tie(lat2, lon2) = (*this->data)[this->start];
    // Divide by 50 since we need an admissible heuristic that doesn't
    // over-estimate, and 50 is optimistic (since in most of the city
    // we have 30 and 50 maxspeed).
    // TODO: However on highways this is not optimistic anymore.
    // TODO: It seems this wasn't working with 50. 160 on the other hand works quite nicely.
    return aerial_dist(lat1, lon1, lat2, lon2) / 160; // Hours
}

std::pair<float, float> cpp_DStar_Lite::calculate_key(long node) {
    float key = std::min(this->g[node], this->rhs[node]);
    return std::pair<float, float>(key + this->heuristic(node) + this->k, key);
}

int cpp_DStar_Lite::update_vertex(long node) {
    if (node != this->goal)
        this->rhs[node] = this->get_min_successor(node).second;

    this->U.erase(node);

    if (this->g[node] != this->rhs[node])
        this->U[node] = this->calculate_key(node);

    return 0;
}

int cpp_DStar_Lite::compute_shortest_path() {
    while (!this->U.empty()) {
        auto candidate = *std::min_element(
                this->U.begin(),
                this->U.end(),
                [](auto& a, auto& b) { return a.second < b.second; });

        bool reached_start = candidate.second >= this->calculate_key(this->start);
        bool start_consistent = this->rhs[this->start] == this->g[this->start];
        if (reached_start && start_consistent)
            break;

        long node = candidate.first;

        std::pair<float, float> k_old = candidate.second;

        this->U.erase(node);
        std::pair<float, float> key = this->calculate_key(node);

        if (k_old < key) {
            this->U[node] = key;
        } else if (this->g[node] > this->rhs[node]) {
            this->g[node] = this->rhs[node];
        } else {
            this->g[node] = INFINITY;
            this->update_vertex(node);
        }

        for (auto &pred : (*this->predecessors)[node]) {
            this->update_vertex(pred);
        }
    }

    return 0;
}

int cpp_DStar_Lite::drive(std::vector<long> &out, google::dense_hash_map<long, long> &diverge_policy) {
    long last_start = this->start;

    std::unordered_set<long> visited = {this->start};
    out.push_back(this->start);

    while (this->start != this->goal) {
        if (this->g[this->start] == INFINITY)
            throw std::runtime_error("No path found.");

        long diverged_node = diverge_policy[this->start];
        if (diverged_node == 0 || visited.find(diverged_node) != visited.end()) {
            this->start = this->get_min_successor(this->start).first;
        } else {
            this->start = diverged_node;

            float lat1, lon1, lat2, lon2;
            std::tie(lat1, lon1) = (*this->data)[last_start];
            std::tie(lat2, lon2) = (*this->data)[this->start];
            // TODO / 50 ?
            // TODO Just use this->heuristic(last_start) ..
            this->k += aerial_dist(lat1, lon1, lat2, lon2) / 50;
            last_start = this->start;
            this->compute_shortest_path();
        }

        visited.insert(this->start);
        out.push_back(this->start);
    }

    return 0;
}
