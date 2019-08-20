#include <cmath>
#include "cpp_mdp.hpp"
#include "../cpp_lib.hpp" // combinations
#include "../serialize_util.hpp"


// Cython needs an integer return, thus all void functions return 0.
// Best viewed with TagList plugin (with vim). :TlistOpen


MDP::MDP() {}
MDP::~MDP() {}


int MDP::init(
        std::vector<long> *S,
        google::dense_hash_map<long, std::vector<std::pair<long, long>>> *A,
        google::dense_hash_map<std::pair<long, long>, double, pair_hash> *C,
        google::dense_hash_map<
            std::pair<long, long>,
            std::vector<std::pair<long, double>>,
            pair_hash
        > *P,
        google::dense_hash_map<std::pair<long, long>, double, pair_hash> *edge_data,
        google::dense_hash_map<long, std::pair<double, double>> *node_data,
        google::dense_hash_map<long, std::vector<long>> *successors,
        google::dense_hash_map<long, std::vector<long>> *predecessors) {
    this->S = S;
    this->A = A;
    this->C = C;
    this->P = P;
    this->edge_data = edge_data;
    this->node_data = node_data;
    this->successors = successors;
    this->predecessors = predecessors;

    this->V.set_empty_key({0, 0});
    this->policy.set_empty_key({0, 0});

    return 0;
}

int MDP::setup(const long &start, const long &goal, std::unordered_map<std::string, double> cfg) {
    this->start = start;
    this->goal = goal;

    this->edge_uncertainty = cfg["edge_uncertainty"];

    this->make_goal_self_absorbing();
    this->make_low_angle_intersections_uncertain(cfg["max_angle"]);
    this->make_close_intersections_uncertain(cfg["max_length"]);

    const int total_uncertain_nodes = this->angle_nodes.size() + this->close_intersections.size();
    const double uncertainty_percent = double(total_uncertain_nodes) / (*this->S).size();
    std::cout << uncertainty_percent * 100 << "\% of nodes are uncertain." << std::endl;

    return 0;
}

int MDP::make_goal_self_absorbing() {
#ifdef TESTS
    save_mdp(this, "MDPmake_goal_self_absorbing.cereal");
#endif

    // Get rid of all leaking actions
    // TODO Needed?
    //std::vector<std::pair<long, long>>::iterator action = (*this->A)[this->goal].begin();
    //while (action != (*this->A)[this->goal].end()) {
    //    if (action->second != this->goal) {
    //        action = (*this->A)[this->goal].erase(action);
    //        // TODO: Erase from P too?
    //    }
    //}

    // Add a zero-cost loop at the goal to absorb cost.
    std::pair<long, long> edge(this->goal, this->goal);
    (*this->A)[this->goal].push_back(edge);
    (*this->C)[edge] = 0;
    (*this->P)[edge] = {{this->goal, 1.0}};

#ifdef TESTS
    save_mdp(this, "MDPmake_goal_self_absorbingWANT.cereal");
#endif
    return 0;
}

int MDP::make_edge_uncertain(
        google::dense_hash_map<
            std::pair<long, long>,
            std::vector<std::pair<long, double>>,
            pair_hash
        > &temp_P,
        const std::pair<long, long> &edge,
        const long &other_node,
        const double &uncertainty) {
    // Make taking the action of following given edge probabilistic,
    // i.e. end up in the expected edge only 90% of the time and end
    // up in other_node 10% of the time.
    // If the edge is already probabilistic, decrease its chance (e.g.
    // from 90% to 80% and so on).
    // Modifies temp_P inplace.
    //
    // Defaults:
    //  uncertainty = .1
    const long node_to = edge.second;

    if (node_to == this->goal || edge.first == this->goal)
        return 0;

    if (temp_P.find(edge) == temp_P.end()) {
        temp_P[edge] = {{node_to, 1 - uncertainty}, {other_node, uncertainty}};
    } else {
        temp_P[edge].push_back({other_node, uncertainty});
        temp_P[edge][0] = {node_to, temp_P[edge][0].second - uncertainty};
    }

    this->uncertain_nodes.insert(edge.first);

    return 0;
}

int MDP::get_normal_intersections(
        google::dense_hash_map<
            std::pair<long, long>,
            Intersection,
            pair_hash
        > &out) {
    // Scan graph for intersections satisfying the following condition:
    // 
    // A node with >= 2 outgoing edges is needed with the following
    // angles:
    //     * 180
    //     * 90 or 270
    // 
    // Returns a list of nodes that satisfy the condition.
    out.set_empty_key(std::pair<long, long>(0, 0));

    // Maximum deviation in degrees before an angle between two edges is not
    // considered square (90, 180, 270) any longer.
    const double max_angle_deviation = 10;

    for (const auto &a : *this->A) {
        for (const auto &origin_edge : a.second) {
            const long origin_node = origin_edge.first;
            const long middle_node = origin_edge.second;

            // Let x be our middle_node and (x, y) our origin_edge. We're
            // coming from the origin_edge.
            //       y2
            //       |
            // y1 ---y--- y3
            //       |
            //       x
            // We now check for all outgoing edges from middle_node whether
            // their angle to (x, y) is one of [90, 180, 270].
            // Example: Angle between (x, y) and (y1, y) is 90.

            long straight_on_node = 0; // y2
            long left_node = 0; // y1
            long right_node = 0; // y3

            for (const auto &succ : (*this->successors)[middle_node]) {
                if (succ == origin_node)
                    continue;

                double p1_x, p1_y, p2_x, p2_y, origin_x, origin_y;

                std::tie(p1_x, p1_y) = (*this->node_data)[origin_node];
                std::tie(p2_x, p2_y) = (*this->node_data)[succ];
                std::tie(origin_x, origin_y) = (*this->node_data)[middle_node];

                double angle = get_angle(p1_x, p1_y, p2_x, p2_y, origin_x, origin_y);

                if (angle < 0)
                    angle += 360;

                if (abs(angle - 90) < max_angle_deviation)
                    left_node = succ;

                if (abs(angle - 270) < max_angle_deviation)
                    right_node = succ;

                if (abs(angle - 180) < max_angle_deviation)
                    straight_on_node = succ;
            }

            if (straight_on_node != 0 && (right_node != 0 || left_node != 0)) {
                const Intersection intersection_out {
                    left_node,
                    right_node,
                    straight_on_node,
                    origin_edge
                };
                out[intersection_out.origin_edge] = intersection_out;
            }
        }
    }

#ifdef TESTS
    {
        std::ofstream os("MDPget_normal_intersectionsWANT.cereal", std::ios::binary);
        cereal::BinaryOutputArchive archive(os);
        archive(out);
    }
#endif

    return 0;
}

int MDP::make_intersection_uncertain(
        const Intersection &intersection,
        const long &intersection_node)
{
    auto origin_node = intersection.origin_edge.second;

    // Driver exits too early.
    this->make_edge_uncertain(
            (*this->P),
            {intersection.origin_edge.second, intersection.straight_on_node},
            intersection_node,
            this->edge_uncertainty);

    // Driver misses exit.
    this->make_edge_uncertain(
            (*this->P),
            {origin_node, intersection_node},
            intersection.straight_on_node,
            this->edge_uncertainty);

    this->close_intersections.insert(intersection);

    return 0;
}

int MDP::make_close_intersections_uncertain(const double &max_length) {
    // Scan graph for intersections that follow very closely.
    // 
    // Use cases:
    //     - When you're supposed to go to the right or left, but go straight
    //       on because you've missed the intersection as it's very close.
    //     - When you're supposed to go straight on so that you can turn right
    //       or left on the next intersection, but you do so on the current
    //       one, which is too early.
    //
    // Defaults:
    //  max_length = 100
#ifdef TESTS
    save_mdp(this, "MDPmake_close_intersections_uncertain.cereal");
#endif
    google::dense_hash_map<std::pair<long, long>, Intersection, pair_hash> intersections;
    this->get_normal_intersections(intersections);

    for (const auto &intersection : intersections) {
        const auto next_edge = std::pair<long, long>(
            intersection.second.origin_edge.second,
            intersection.second.straight_on_node);

        Intersection next_intersection;

        if (intersections.find(next_edge) != intersections.end()) {
            next_intersection = intersections[next_edge];
        } else {
            continue;
        }

        const double next_edge_length = (*this->edge_data)[next_edge];

        if (next_edge_length > max_length)
            continue;

        if (intersection.second.left_node != 0 && next_intersection.left_node != 0)
            this->make_intersection_uncertain(
                    intersection.second,
                    intersection.second.left_node);

        if (intersection.second.right_node != 0 && next_intersection.right_node != 0)
            this->make_intersection_uncertain(
                    intersection.second,
                    intersection.second.right_node);
    }

#ifdef TESTS
    save_mdp(this, "MDPmake_close_intersections_uncertainWANT.cereal");
#endif

    return 0;
}

int MDP::make_low_angle_intersections_uncertain(const double &max_angle) {
    //  (2)   (3)
    //   *     *
    //    \   /
    //     \ /
    //      * (1)
    // 
    // If angle between edges (1, 2) and (1, 3) is small enough,
    // make those edges uncertain, i.e. add a 10% chance end up
    // in the other node and not the expected one.
    //
    // Defaults:
    //  max_angle = 30
#ifdef TESTS
    save_mdp(this, "MDPmake_low_angle_intersections_uncertain.cereal");
#endif

    // TODO: Twisted code. Works, but succs
    for (const auto &s : *this->S) {
        for (const auto &succ : (*this->successors)[s]) {
            std::pair<long, long> edge(s, succ);
            const long origin_node = edge.second;

            int n_critical_nodes = 0;

            google::dense_hash_map<std::pair<long, long>, std::vector<std::pair<long, double>>, pair_hash> temp_P;
            temp_P.set_empty_key({0, 0});

            double x3, y3, origin_x, origin_y;

            std::tie(x3, y3) = (*this->node_data)[edge.first];
            std::tie(origin_x, origin_y) = (*this->node_data)[edge.second];

            const auto combs = combinations(origin_node, (*this->successors)[origin_node]);

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

                double x1, y1, x2, y2;

                std::tie(x1, y1) = (*this->node_data)[edge1.second];
                std::tie(x2, y2) = (*this->node_data)[edge2.second];

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
                const double angle = get_angle(x1, y1, x3, y3, origin_x, origin_y) - get_angle(x2, y2, x3, y3, origin_x, origin_y);
                if (abs(angle) <= max_angle) {
                    make_edge_uncertain(temp_P, edge1, edge2.second, this->edge_uncertainty);
                    make_edge_uncertain(temp_P, edge2, edge1.second, this->edge_uncertainty);

                    n_critical_nodes += 1;
                }
            }

            if (n_critical_nodes > 0)
                this->angle_nodes.push_back(origin_node);

            for (auto &kv : temp_P) {
                (*P)[kv.first] = kv.second;
            }
        }
    }

#ifdef TESTS
    save_mdp(this, "MDPmake_low_angle_intersections_uncertainWANT.cereal");
#endif
    
    return 0;
}

// TODO Same as BRTDP
double MDP::get_Q_value(
        google::dense_hash_map<std::pair<long, long>, double, pair_hash> &prev_V,
        const std::pair<long, long> &s_pair,
        const std::pair<long, long> &a)
{
    double future_cost = 0;
    for (const auto &outcome : (*this->P)[a]) {
        std::pair<long, long> node_pair = {s_pair.second, outcome.first};
        future_cost += outcome.second * ((*this->C)[node_pair] + prev_V[node_pair]);
    }

    if (a.second == s_pair.first) {
        future_cost += U_TURN_PENALTY;
    }

    return future_cost;
}

// TODO Hashmaps of Hashmaps sucks, change this.
auto MDP::init_Q() {
    google::dense_hash_map<
        std::pair<long, long>,
        google::dense_hash_map<std::pair<long, long>, double, pair_hash>,
        pair_hash> Q;
    Q.set_empty_key({0, 0});

    for (auto &s : *this->S) {
        for (auto &pred : (*this->predecessors)[s]) {
            std::pair<long, long> state_pair = {pred, s};

            Q[state_pair] = google::dense_hash_map<std::pair<long, long>, double, pair_hash>();
            Q[state_pair].set_empty_key(std::pair<long, long>(0, 0));

            for (auto &a : (*this->A)[s]) {
                Q[state_pair][a] = 0.;
            }

            V[state_pair] = 0.;
        }
    }

    return Q;
}

bool MDP::converged(
        google::dense_hash_map<std::pair<long, long>, double, pair_hash> &prev_V,
        const double &eps)
{
    double c = 0;

    for (auto &v : this->V) {
        c += v.second - prev_V[v.first];
    }

    return c < eps;
}

int MDP::solve(const int &max_iter, const double &eps) {
    // Solve the MDP with value iteration
    // Defaults:
    //  max_iter = 50000
    //  eps = 1e-20
#ifdef TESTS
    save_mdp(this, "MDPsolve.cereal");
#endif

    auto Q = this->init_Q();

    int i = 0;
    for (; i < max_iter; ++i) {
        auto prev_V = V;

        for (const auto &s : *this->S) {
            for (auto &pred : (*this->predecessors)[s]) {
                std::pair<long, long> state_pair = {pred, s};

                for (const auto &a : (*this->A)[s]) {
                    Q[state_pair][a] = this->get_Q_value(prev_V, state_pair, a);
                }

                std::pair<const std::pair<long, long>, double> best_action = *min_element(
                        Q[state_pair].begin(),
                        Q[state_pair].end(),
                        [](auto& a, auto& b) { return a.second < b.second; });

                this->V[state_pair] = best_action.second;
            }
        }

        // Only check for convergence every 10 (100) runs.
        // TODO: Does this actually improve performance?
        if (i % 10 == 0) {
            if (this->converged(prev_V, eps))
                break;
        }
    }

    std::cout << "ITERATIONS: " << i << std::endl;

#ifdef TESTS
    save_mdp(this, "MDPsolveWANT.cereal");
#endif

    // Cython needs an integer return.
    return i;
}

int MDP::get_policy() {
#ifdef TESTS
    save_mdp(this, "MDPget_policy.cereal");
#endif

    for (auto &s : *this->S) {
        for (auto &pred : (*this->predecessors)[s]) {
            double curr_min = INFINITY;
            std::pair<long, long> curr_min_action;

            for (auto &a : (*this->A)[s]) {
                // TODO Use Q value function here. Code duplication.
                double cost = 0;
                for (auto &outcome : (*this->P)[a]) {
                    cost += outcome.second * ((*this->C)[a] + V[{s, outcome.first}]);
                }

                if (a.second == pred) {
                    cost += U_TURN_PENALTY;
                }

                if (cost < curr_min) {
                    curr_min = cost;
                    curr_min_action = a;
                }
            }

            policy[{pred, s}] = curr_min_action;
        }
    }

#ifdef TESTS
    save_mdp(this, "MDPget_policyWANT.cereal");
#endif

    return 0;
}

std::vector<long> MDP::drive(google::dense_hash_map<long, long> &diverge_policy) {
#ifdef TESTS
    save_mdp(this, "MDPdrive.cereal");
#endif
    // Make sure we don't loop indefinitely due to diverge policy
    std::unordered_set<long> visited;

    std::vector<long> nodes = {this->start};
    std::pair<long, long> curr_node = {(*this->predecessors)[this->start][0], this->start};

    while (curr_node.second != this->goal) {
        if (policy.find(curr_node) == policy.end())
            break;

        const long diverged_node = diverge_policy[curr_node.second];
        if (diverged_node == 0 || visited.find(diverged_node) != visited.end()) {
            curr_node = policy[curr_node];
        } else {
            curr_node = {curr_node.second, diverged_node};
            visited.insert(diverged_node);
        }

        nodes.push_back(curr_node.second);
    }

#ifdef TESTS
    {
        std::ofstream os("MDPdriveWANT.cereal", std::ios::binary);
        cereal::BinaryOutputArchive archive(os);
        archive(nodes);
    }
#endif

    return nodes;
}
