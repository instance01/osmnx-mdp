#include "cpp_dstar_lite.hpp"
#include <cmath> // sin, asin, cos, pow, M_PI
#include <algorithm> // min
#include <stdexcept> // runtime_error

#include <iostream> // TODO REMOVE

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

cpp_DStar_Lite::cpp_DStar_Lite () {
}

cpp_DStar_Lite::~cpp_DStar_Lite () {
}

// TODO Lmao reorder args
int cpp_DStar_Lite::init(
        google::dense_hash_map<std::pair<long, long>, float, pair_hash> *cost,
        google::dense_hash_map<long, std::pair<float, float>> *data,
        google::dense_hash_map<long, std::vector<long>> *predecessors,
        google::dense_hash_map<long, std::vector<long>> *successors) {
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

// TODO Move into lib.cpp
float aerial_dist(float lat1, float lon1, float lat2, float lon2, float R=6356.8) {
    lon1 *= M_PI / 180.0;
    lon2 *= M_PI / 180.0;
    lat1 *= M_PI / 180.0;
    lat2 *= M_PI / 180.0;
    float d = pow(sin((lat2 - lat1) / 2), 2) + cos(lat1) * cos(lat2) * pow(sin((lon2 - lon1) / 2), 2);
    return R * 2 * asin(pow(d, .5));
}

float cpp_DStar_Lite::heuristic_func(long node) {
    float lat1, lon1, lat2, lon2;
    lat1 = (*this->data)[node].first;
    lon1 = (*this->data)[node].second;
    std::tie(lat1, lon1) = (*this->data)[node];
    std::tie(lat2, lon2) = (*this->data)[this->start];
    // Divide by 50 since we need an admissible heuristic that doesn't
    // over-estimate, and 50 is optimistic (since in most of the city
    // we have 30 and 50 maxspeed).
    // TODO: However on highways this is not optimistic anymore.
    return aerial_dist(lat1, lon1, lat2, lon2) / 50; // Hours
}

std::pair<float, float> cpp_DStar_Lite::calculate_key(long node) {
    float key = std::min(this->g[node], this->rhs[node]);
    return std::pair<float, float>(key + this->heuristic_func(node) + this->k, key);
}

int cpp_DStar_Lite::update_vertex(long u) {
    if (u != this->goal) {
        // TODO: Too manual, improve
        float curr_min = INFINITY;
        for (auto &succ : (*this->successors)[u]) {
            float cost = this->g[succ] + (*this->cost)[std::pair<long, long>(u, succ)];
            curr_min = std::min(cost, curr_min);
        }
        this->rhs[u] = curr_min;
    }

    this->U.erase(u);

    if (this->g[u] != this->rhs[u]) {
        this->U[u] = this->calculate_key(u);
    }

    return 0;
}

int cpp_DStar_Lite::compute_shortest_path() {
    while (!this->U.empty() &&
            ((*std::min_element(this->U.begin(), this->U.end(), [](auto& a, auto& b) { return a.second < b.second; })).second < this->calculate_key(this->start) ||
             this->rhs[this->start] != this->g[this->start])) {
        long u = (*std::min_element(this->U.begin(), this->U.end(), [](auto& a, auto& b) { return a.second < b.second; })).first;
        std::pair<float, float> k_old = this->U[u];
        this->U.erase(u);

        std::pair<float, float> key = this->calculate_key(u);
        if (k_old < key) {
            this->U[u] = key;
        } else if (this->g[u] > this->rhs[u]) {
            this->g[u] = this->rhs[u];
        } else {
            this->g[u] = INFINITY;
            this->update_vertex(u);
        }

        for (auto &node : (*this->predecessors)[u]) {
            this->update_vertex(node);
        }
    }
    return 0;
}

// TODO Rename visited to old
// TODO: Pass diverge_policy by reference
int cpp_DStar_Lite::drive(std::set<long> &visited, google::dense_hash_map<long, long> diverge_policy) {
    long last_start = this->start;

    visited.insert(this->start);

    while (this->start != this->goal) {
        if (this->g[this->start] == INFINITY)
            throw std::runtime_error("No path found.");

        long diverged_node = diverge_policy[this->start];
        if (diverged_node == 0 || visited.find(diverged_node) != visited.end()) {
            this->start = *std::min_element(
                    (*this->successors)[this->start].begin(),
                    (*this->successors)[this->start].end(),
            [this](auto& a, auto& b) {
                float a_cost = this->g[a] + (*this->cost)[std::pair<long, long>(this->start, a)];
                float b_cost = this->g[b] + (*this->cost)[std::pair<long, long>(this->start, b)];
                return a_cost < b_cost;
            });

            visited.insert(this->start);
        } else {
            visited.insert(diverged_node);
            this->start = diverged_node;

            float lat1, lon1, lat2, lon2;
            std::tie(lat1, lon1) = (*this->data)[last_start];
            std::tie(lat2, lon2) = (*this->data)[this->start];
            // TODO / 50 ?
            this->k += aerial_dist(lat1, lon1, lat2, lon2) / 50;
            last_start = this->start;
            this->compute_shortest_path();
        }
    }

    return 0;
}
