#include "cpp_mdp.hpp"
#include <fstream>
#include <iostream> // cout
#include <algorithm> // min_element
#include <math.h>


float get_angle(
        float p1_x,
        float p1_y,
        float p2_x,
        float p2_y,
        float origin_x,
        float origin_y) {
    p1_x -= origin_x;
    p1_y -= origin_y;
    p2_x -= origin_x;
    p2_y -= origin_y;

    // arctan2 gives us the angle between the ray from the origin to the point
    // and the x axis.
    // Thus, to get the angle between two points, simply get the difference
    // between their two angles to the x axis.
    float angle = atan2(p2_x, p2_y) - atan2(p1_x, p1_y);
    return 180 * angle / M_PI;
}

int CPP_get_normal_intersections(
        google::dense_hash_map<std::pair<long, long>, CPP_Intersection, pair_hash> &out,
        google::dense_hash_map<long, std::vector<long>> &successors,
        google::dense_hash_map<long, std::pair<float, float>> &data) {
    // TODO: Twisted code, improve
    out.set_empty_key(std::pair<long, long>(0, 0));
    for (auto &x : successors) {
        for (auto &succ : x.second) {
            long origin_node = succ;

            long straight_on_node = 0;
            long left_node = 0;
            long right_node = 0;

            for (auto &succsucc : successors[succ]) {
                float p1_x, p1_y, p2_x, p2_y, origin_x, origin_y;

                std::tie(p1_x, p1_y) = data[x.first];
                std::tie(p2_x, p2_y) = data[succsucc];
                std::tie(origin_x, origin_y) = data[origin_node];

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

void make_edge_uncertain(
        google::dense_hash_map<std::pair<long, long>, std::vector<std::pair<long, double>>, pair_hash> &temp_P,
        const std::pair<long, long> &edge,
        const long &other_node) {
    // TODO: Add docstring from python
    long node_to = edge.second;

    if (temp_P.find(edge) == temp_P.end()) {
        temp_P[edge] = {{node_to, .9}, {other_node, .1}};
    } else {
        temp_P[edge].push_back({other_node, .1});
        temp_P[edge][0] = {node_to, temp_P[edge][0].second - .1};
    }
}

int CPP_make_low_angle_intersections_uncertain(
        std::vector<long> *angle_nodes,
        google::dense_hash_map<
            long,
            google::dense_hash_map<
                std::pair<long, long>,
                std::vector<std::pair<long, double>>,
                pair_hash
            >
        > *P,
        google::dense_hash_map<long, std::vector<long>> &successors,
        google::dense_hash_map<long, std::pair<float, float>> &data,
        float max_angle=30) {
    // TODO: Twisted code. Works, but succs
    // TODO: Add all docstrings/comments from python code
    for (auto &x : data) {
        for (auto &succ : successors[x.first]) {
            std::pair<long, long> edge(x.first, succ);
            long origin_node = edge.second;

            int n_critical_nodes = 0;

            google::dense_hash_map<std::pair<long, long>, std::vector<std::pair<long, double>>, pair_hash> temp_P;
            temp_P.set_empty_key({0, 0});

            float x3 = data[edge.first].first;
            float y3 = data[edge.first].second;

            float origin_x = data[edge.second].first;
            float origin_y = data[edge.second].second;

            std::vector<std::pair<std::pair<long, long>, std::pair<long, long>>> combs;
            combinations(combs, origin_node, successors[origin_node]);

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

                float x1 = data[edge1.second].first;
                float y1 = data[edge1.second].second;
                float x2 = data[edge2.second].first;
                float y2 = data[edge2.second].second;

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
                angle_nodes->push_back(origin_node);

            for (auto &kv : temp_P) {
                (*P)[succ][kv.first] = kv.second;
            }
        }
    }
    
    return 0;
}

int solve(
        google::dense_hash_map<long, float> &V,
        std::vector<long> &S,
        google::dense_hash_map<long, std::vector<std::pair<long, long>>> &A,
        google::dense_hash_map<std::pair<long, long>, float, pair_hash> &C,
        google::dense_hash_map<
            long,
            google::dense_hash_map<
                std::pair<long, long>,
                std::vector<std::pair<long, float>>,
                pair_hash
            >
        > &P,
        int max_iter=10000) {
    google::dense_hash_map<long, google::dense_hash_map<std::pair<long, long>, float, pair_hash>> Q;
    Q.set_empty_key(0);
    for (auto &s : S) {
        Q[s] = google::dense_hash_map<std::pair<long, long>, float, pair_hash>();
        Q[s].set_empty_key(std::pair<long, long>(0, 0));
        for (auto &a : A[s]) {
            Q[s][a] = 0.;
        }
        V[s] = 0.;
    }

    float gamma = 1.;
    for (int i = 0; i < max_iter; ++i) {
        google::dense_hash_map<long, float> prev_V = V;

        for (auto &s : S) {
            for (auto &a : A[s]) {
                float immediate_cost = C[a];
                float future_cost = 0;
                for (auto &outcome : P[s][a]) {
                    future_cost += outcome.second * gamma * prev_V[outcome.first];
                }
                Q[s][a] = immediate_cost + future_cost;
            }

            //if (Q[s].empty())
            //if (Q.find(s) == Q.end()) {
            //    std::cout << A[s].empty() << std::endl;
            //    continue;
            //}

            std::pair<const std::pair<long, long>, float> best_action = *min_element(
                    Q[s].begin(),
                    Q[s].end(),
                    [](auto& a, auto& b) { return a.second < b.second; });

            V[s] = best_action.second;
        }


        if (i % 100 == 0) {
            float c = 0;
            auto V_iter = V.begin();
            auto prev_V_iter = prev_V.begin();
            while (V_iter != V.end() || prev_V_iter != prev_V.end()) {
                c += (*V_iter).second - (*prev_V_iter).second;
                ++V_iter;
                ++prev_V_iter;
            }
            std::cout << c << std::endl;
            if (c < 1e-30)
                break;
        }
    }

    // Cython needs an integer return.
    return 0;
}
