#include <cmath> // sin, asin, cos, pow, M_PI
#include <cfloat> // DBL_EPSILON
#include <algorithm> // min_element
#include <stdexcept> // runtime_error
#include <unordered_set>

#include "cpp_dstar_lite.hpp"

#include "../serialize_util.hpp"
#include "../cpp_queue_util.hpp" // queue_decrease_priority, queue_pop


// TODO Rename DStar_Lite

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

    this->k = 0;

    if (cfg["heuristic"] == 0) {
        this->dijkstra_heuristic = true;
        this->init_heuristic();
    } else {
        this->dijkstra_heuristic = false;
    }

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

std::pair<long, double> DStar_Lite::get_min_successor(const std::pair<long, long> &node_pair)
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
        double cost = this->g[succ] + edge_cost;

        // Penalize U-turns by increasing cost by 10%.
        if (node_pair.first == succ) {
            cost *= 1.1;
        }

        if (cost <= min_cost) {
            min_cost = cost;
            min_node = succ;
        }
    }

    return {min_node, min_cost};
}

float DStar_Lite::aerial_heuristic(const long &node)
{
    float lat1, lon1, lat2, lon2;
    std::tie(lat1, lon1) = (*this->data)[node];
    std::tie(lat2, lon2) = (*this->data)[this->start];
    return aerial_dist(lat1, lon1, lat2, lon2) / 200; // Hours
}

std::pair<double, double> DStar_Lite::calculate_key(const long &node)
{
    double key = std::min(this->g[node], this->rhs[node]);
    double heuristic_val = this->dijkstra_heuristic ? this->heuristic_map[node] : aerial_heuristic(node);
    return std::pair<double, double>(key + heuristic_val + this->k, key);
}

int DStar_Lite::update_vertex(const long &node)
{
    if (this->g[node] != this->rhs[node])
        this->U[node] = this->calculate_key(node);
    else if(this->g[node] == this->rhs[node])
        this->U.erase(node);

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
        // TODO: Different from the optimized version of D* Lite
        //const bool start_not_overconsistent = this->rhs[this->start] <= this->g[this->start];
        //if (reached_start && start_not_overconsistent)
        //    break;

        const long node = candidate.first;
        const auto k_old = candidate.second;

        const auto key = this->calculate_key(node);

        if (k_old < key) {
            this->U[node] = key;
        } else if (this->g[node] > this->rhs[node]) {
            this->g[node] = this->rhs[node];
            this->U.erase(node);
            for (const auto &pred : (*this->predecessors)[node]) {
                if (pred != this->goal)
                    this->rhs[pred] = std::min(
                            this->rhs[pred],
                            (*this->cost)[{pred, node}] + this->g[node]);
                this->update_vertex(pred);
            }
        } else {
            const auto g_old = this->g[node];
            this->g[node] = INFINITY;

            // TODO: This might potentially rather SLOW it down than make it faster..
            auto to_update = (*this->predecessors)[node];
            to_update.push_back(node);
            for (const auto &x : to_update) {
                if (this->rhs[x] == (*this->cost)[{x, node}] + g_old) {
                    if (x != this->goal) {
                        //this->rhs[x] = this->get_min_successor({node, x}).second;
                        this->rhs[x] = this->get_min_successor({0, x}).second;
                    }
                }
                this->update_vertex(x);
            }
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
    std::pair<long, long> start_pair = {0, this->start};
    long last_start = this->start;

    std::unordered_set<std::pair<long, long>, pair_hash> visited = {start_pair};
    out.push_back(this->start);

    while (this->start != this->goal) {
        if (this->g[this->start] == INFINITY)
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
    }
    dist[this->goal] = 0;
    queue.push_back({this->goal, dist[this->goal]});

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
