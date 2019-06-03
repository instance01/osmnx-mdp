#include <iostream> // cout
#include <algorithm> // min_element
#include <set>
#include <math.h>

#include "cpp_mdp.hpp"


void combinations(
        std::vector<std::pair<std::pair<long, long>, std::pair<long, long>>> &out,
        long origin_node,
        std::vector<long> successors) {
    for (unsigned long i = 0; i < successors.size(); ++i) {
        for (unsigned long j = 0; j < successors.size(); ++j) {
            if (i >= j)
                continue;
            std::pair<long, long> edge1(origin_node, successors[i]);
            std::pair<long, long> edge2(origin_node, successors[j]);
            out.push_back(std::pair<std::pair<long, long>, std::pair<long, long>>(edge1, edge2));
        }
    }
}


CPP_MDP::CPP_MDP() {}
CPP_MDP::~CPP_MDP() {}


int CPP_MDP::init(
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
    google::dense_hash_map<std::pair<long, long>, double, pair_hash> *edge_data,
    google::dense_hash_map<long, std::pair<double, double>> *node_data,
    google::dense_hash_map<long, std::vector<long>> *successors) {
    this->S = S;
    this->A = A;
    this->C = C;
    this->P = P;
    this->edge_data = edge_data;
    this->node_data = node_data;
    this->successors = successors;

    return 0;
}

int CPP_MDP::setup(long start, long goal) {
    this->start = start;
    this->goal = goal;

    this->make_goal_self_absorbing();
    this->make_low_angle_intersections_uncertain();
    this->make_close_intersections_uncertain();
 
    int total_uncertain_nodes = this->angle_nodes.size() + this->close_intersections.size();
    float uncertainty_percent = float(total_uncertain_nodes) / (*this->S).size();//13970.;//this->G.number_of_nodes() * 100;
    std::cout << uncertainty_percent * 100 << "\% of nodes are uncertain." << std::endl;
    return 0;
}

int CPP_MDP::make_goal_self_absorbing() {
    // Add a zero-cost loop at the goal to absorb cost.
    std::pair<long, long> edge(this->goal, this->goal);
    (*this->A)[this->goal].push_back(edge);
    (*this->C)[edge] = 0;
    (*this->P)[this->goal][edge] = {{this->goal, 1.0}};

    return 0;
}

int CPP_MDP::make_edge_uncertain(
        google::dense_hash_map<
            std::pair<long, long>,
            std::vector<std::pair<long, double>>,
            pair_hash
        > &temp_P,
        const std::pair<long, long> &edge,
        const long &other_node) {
    long node_to = edge.second;

    if (temp_P.find(edge) == temp_P.end()) {
        temp_P[edge] = {{node_to, .9}, {other_node, .1}};
    } else {
        temp_P[edge].push_back({other_node, .1});
        temp_P[edge][0] = {node_to, temp_P[edge][0].second - .1};
    }

    return 0;
}

int CPP_MDP::get_normal_intersections(
        google::dense_hash_map<std::pair<long, long>, CPP_Intersection, pair_hash> &out) {
    // Scan graph for intersections satisfying the following condition:
    // 
    // A node with >= 2 outgoing edges is needed with the following
    // angles:
    //     * 180
    //     * 90 or 270
    // 
    // Returns a list of nodes that satisfy the condition.
    // TODO: Twisted code, improve
    out.set_empty_key(std::pair<long, long>(0, 0));
    for (auto &x : *this->successors) {
        for (auto &succ : x.second) {
            long origin_node = succ;

            long straight_on_node = 0;
            long left_node = 0;
            long right_node = 0;

            for (auto &succsucc : (*this->successors)[succ]) {
                double p1_x, p1_y, p2_x, p2_y, origin_x, origin_y;

                std::tie(p1_x, p1_y) = (*this->node_data)[x.first];
                std::tie(p2_x, p2_y) = (*this->node_data)[succsucc];
                std::tie(origin_x, origin_y) = (*this->node_data)[origin_node];

                float angle = get_angle(p1_x, p1_y, p2_x, p2_y, origin_x, origin_y);

                if (angle < 0)
                    angle += 360;

                if (abs(angle - 90) < 10)
                    left_node = succsucc;

                if (abs(angle - 270) < 10)
                    right_node = succsucc;

                if (abs(angle - 180) < 10)
                    straight_on_node = succsucc;
            }

            if (straight_on_node != 0 && (right_node != 0 || left_node != 0)) {
                CPP_Intersection intersection;
                intersection.left_node = left_node;
                intersection.right_node = right_node;
                intersection.straight_on_node = straight_on_node;
                intersection.origin_edge = std::pair<long, long>(x.first, origin_node);

                out[intersection.origin_edge] = intersection;
            }
        }
    }

    return 0;
}

int CPP_MDP::make_close_intersections_uncertain(float max_length) {
    // Scan graph for intersections that follow very closely.
    // 
    // Use cases:
    //     - When you're supposed to go to the right or left, but go straight
    //       on because you've missed the intersection as it's very close.
    //     - When you're supposed to go straight on so that you can turn right
    //       or left on the next intersection, but you do so on the current
    //       one, which is too early.
    google::dense_hash_map<std::pair<long, long>, CPP_Intersection, pair_hash> intersections;
    intersections.set_empty_key({0, 0});

    this->get_normal_intersections(intersections);

    for (auto &intersection : intersections) {
        std::pair<long, long> next_edge = std::pair<long, long>(intersection.second.origin_edge.second, intersection.second.straight_on_node);

        CPP_Intersection next_intersection;

        if (intersections.find(next_edge) != intersections.end()) {
            next_intersection = intersections[next_edge];
        } else {
            continue;
        }

        std::pair<long, long> origin_edge = std::pair<long, long>(intersection.second.origin_edge.first, intersection.second.origin_edge.second);
        double origin_edge_length = (*this->edge_data)[origin_edge];
        double next_edge_length = (*this->edge_data)[next_edge];
        // TODO: Consider ox.clean_intersections, then that <20 check isn't
        // needed anymore.
        if (next_edge_length > max_length || origin_edge_length < 20)
            continue;

        long origin_node = origin_edge.second;

        if (intersection.second.left_node != 0 && next_intersection.left_node != 0) {
            this->close_intersections.push_back(intersection.second);
            this->make_edge_uncertain(
                    (*this->P)[origin_node],
                    next_edge,
                    intersection.second.left_node);
            this->make_edge_uncertain(
                    (*this->P)[origin_node],
                    std::pair<long, long>(origin_node, intersection.second.left_node),
                    intersection.second.straight_on_node);
        }

        if (intersection.second.right_node != 0 && next_intersection.right_node != 0) {
            // TODO: Improve code
            if (std::find(
                        this->close_intersections.begin(),
                        this->close_intersections.end(),
                        intersection.second) == this->close_intersections.end()) {
                this->close_intersections.push_back(intersection.second);
            }
            this->make_edge_uncertain(
                    (*this->P)[origin_node],
                    next_edge,
                    intersection.second.right_node);
            this->make_edge_uncertain(
                    (*this->P)[origin_node],
                    std::pair<long, long>(origin_node, intersection.second.right_node),
                    intersection.second.straight_on_node);
        }

    }

    return 0;
}

int CPP_MDP::make_low_angle_intersections_uncertain(float max_angle) {
    //  (2)   (3)
    //   *     *
    //    \   /
    //     \ /
    //      * (1)
    // 
    // If angle between edges (1, 2) and (1, 3) is small enough,
    // make those edges uncertain, i.e. add a 10% chance end up
    // in the other node and not the expected one.

    // TODO: Twisted code. Works, but succs
    // TODO: Add all docstrings/comments from python code
    for (auto &x : *this->node_data) {
        for (auto &succ : (*this->successors)[x.first]) {
            std::pair<long, long> edge(x.first, succ);
            long origin_node = edge.second;

            int n_critical_nodes = 0;

            google::dense_hash_map<std::pair<long, long>, std::vector<std::pair<long, double>>, pair_hash> temp_P;
            temp_P.set_empty_key({0, 0});

            float x3 = (*this->node_data)[edge.first].first;
            float y3 = (*this->node_data)[edge.first].second;

            float origin_x = (*this->node_data)[edge.second].first;
            float origin_y = (*this->node_data)[edge.second].second;

            std::vector<std::pair<std::pair<long, long>, std::pair<long, long>>> combs;
            combinations(combs, origin_node, (*this->successors)[origin_node]);

            for (auto &combination : combs) {
                std::pair<long, long> edge1 = combination.first;
                std::pair<long, long> edge2 = combination.second;

                // We can't reuse the edge that goes back as one of the
                // out_edges.
                // 1   3
                //  \  |
                //   \ |
                //     2
                // We come from 1, and without the checks below we reuse
                // edge (1, 2) and erroneously find a critical angle
                // between (1, 2) and (2, 3).
                if (
                        (edge1.first == edge.second && edge1.second == edge.first) ||
                        (edge2.first == edge.second && edge2.second == edge.first)) {
                    continue;
                }

                float x1 = (*this->node_data)[edge1.second].first;
                float y1 = (*this->node_data)[edge1.second].second;
                float x2 = (*this->node_data)[edge2.second].first;
                float y2 = (*this->node_data)[edge2.second].second;

                // The following line solves the following scenario:
                // 1   3   4
                //  \  |  /
                //   \ | /
                //     2
                // Let's say we come from node 3 and we're currently at node 2.
                // We used to simply get the angle between the two edges (here
                // (1, 2) and (4, 2)), which in this case is <30 degrees.
                // But this is a T-shaped intersection, just sharper. This is
                // not a critical intersection.
                // So currently we get the angle between edge (4, 2) and (3, 2),
                // which is e.g. 20 degrees, and the angle between (1, 2) and
                // (3, 2), which is then 340 degrees.
                // It follows that the difference of both is 320 > 30 degrees.
                // Thus it is not a critical intersection.
                float angle = get_angle(x1, y1, x3, y3, origin_x, origin_y) - get_angle(x2, y2, x3, y3, origin_x, origin_y);
                if (abs(angle) <= max_angle) {
                    make_edge_uncertain(temp_P, edge1, edge2.second);
                    make_edge_uncertain(temp_P, edge2, edge1.second);

                    n_critical_nodes += 1;
                }
            }

            if (n_critical_nodes > 0)
                this->angle_nodes.push_back(origin_node);

            for (auto &kv : temp_P) {
                (*P)[succ][kv.first] = kv.second;
            }
        }
    }
    
    return 0;
}

// TODO Everything needs to be double, not float.
int CPP_MDP::solve(int max_iter, double eps) {
    V.set_empty_key(0);
    google::dense_hash_map<long, google::dense_hash_map<std::pair<long, long>, float, pair_hash>> Q;
    Q.set_empty_key(0);
    for (auto &s : *this->S) {
        Q[s] = google::dense_hash_map<std::pair<long, long>, float, pair_hash>();
        Q[s].set_empty_key(std::pair<long, long>(0, 0));
        for (auto &a : (*this->A)[s]) {
            Q[s][a] = 0.;
        }
        V[s] = 0.;
    }

    for (int i = 0; i < max_iter; ++i) {
        google::dense_hash_map<long, float> prev_V = V;

        for (auto &s : *this->S) {
            for (auto &a : (*this->A)[s]) {
                float immediate_cost = (*this->C)[a];
                float future_cost = 0;
                for (auto &outcome : (*this->P)[s][a]) {
                    future_cost += outcome.second * prev_V[outcome.first];
                }
                Q[s][a] = immediate_cost + future_cost;
            }

            std::pair<const std::pair<long, long>, float> best_action = *min_element(
                    Q[s].begin(),
                    Q[s].end(),
                    [](auto& a, auto& b) { return a.second < b.second; });

            V[s] = best_action.second;
        }

        // Only check for convergence every 100 runs.
        // TODO: Does this actually improve performance?
        if (i % 100 == 0) {
            double c = 0;
            auto V_iter = V.begin();
            auto prev_V_iter = prev_V.begin();

            while (V_iter != V.end() || prev_V_iter != prev_V.end()) {
                c += (*V_iter).second - (*prev_V_iter).second;
                ++V_iter;
                ++prev_V_iter;
            }

            if (c < eps)
                break;
        }
    }

    // Cython needs an integer return.
    return 0;
}

int CPP_MDP::get_policy() {
    policy.set_empty_key(0);
    for (auto &s : *this->S) {
        double curr_min = INFINITY;
        std::pair<long, long> curr_min_action;

        for (auto &a : (*this->A)[s]) {
            double cost = 0;
            for (auto &outcome : (*this->P)[s][a]) {
                cost += outcome.second * ((*this->C)[a] + V[outcome.first]);
            }

            if (cost < curr_min) {
                curr_min = cost;
                curr_min_action = a;
            }
        }

        policy[s] = curr_min_action;
    }

    return 0;
}

std::vector<long> CPP_MDP::drive(google::dense_hash_map<long, long> &diverge_policy) {
    // Make sure we don't loop indefinitely due to diverge policy
    std::set<long> visited;

    std::vector<long> nodes = {this->start};
    std::set<long> nodes_lookup(nodes.begin(), nodes.end());
    long curr_node = this->start;

    while (curr_node != this->goal) {
        if (policy.find(curr_node) == policy.end())
            break;

        long diverged_node = 0;
        if (diverge_policy.find(curr_node) != diverge_policy.end())
            diverged_node = diverge_policy[curr_node];

        if (diverged_node == 0 || visited.find(diverged_node) != visited.end()) {
            curr_node = policy[curr_node].second;
        } else {
            curr_node = diverged_node;
            visited.insert(diverged_node);
        }

        nodes.push_back(curr_node);
    }

    return nodes;
}
