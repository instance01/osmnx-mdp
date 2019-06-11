#include <cmath> // sin, asin, cos, pow, M_PI
#include <algorithm> // min_element
#include <stdexcept> // runtime_error
#include <unordered_set>

#include "cpp_dstar_lite.hpp"

#include "../serialize_util.hpp"


DStar_Lite::DStar_Lite () {}
DStar_Lite::~DStar_Lite () {}

int DStar_Lite::init(
        google::dense_hash_map<long, std::vector<long>> *predecessors,
        google::dense_hash_map<long, std::vector<long>> *successors,
        google::dense_hash_map<std::pair<long, long>, float, pair_hash> *cost,
        google::dense_hash_map<long, std::pair<float, float>> *data)
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

int DStar_Lite::setup(const long &start, const long &goal)
{
    this->start = start;
    this->goal = goal;

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

std::pair<long, float> DStar_Lite::get_min_successor(const long &node)
{
    // For a given node, return the successor which lies on the minimum sum
    // of the estimated path to the node and the edge cost between the node
    // and the successor.

    float min_cost = INFINITY;
    long min_node = 0;

    // Manually find minimum. An alternative would be to use min_element,
    // however it is actually less readable in this case.
    for (const auto &succ : (*this->successors)[node]) {
        const float edge_cost = (*this->cost)[{node, succ}];
        const float cost = this->g[succ] + edge_cost;

        if (cost <= min_cost) {
            min_cost = cost;
            min_node = succ;
        }
    }

    return {min_node, min_cost};
}

// TODO Rename DStar_Lite
// TODO Remove divisor thing
float DStar_Lite::heuristic(const long &node)
{
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

std::pair<float, float> DStar_Lite::calculate_key(const long &node)
{
    float key = std::min(this->g[node], this->rhs[node]);
    return std::pair<float, float>(key + this->heuristic(node) + this->k, key);
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
    while (!this->U.empty()) {
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

    return 0;
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
            this->k += this->heuristic(last_start);
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
