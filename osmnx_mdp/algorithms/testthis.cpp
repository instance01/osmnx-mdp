// #include "sparsepp/spp.h"
#include "cereal/types/unordered_map.hpp"
#include "cereal/types/vector.hpp"
#include "cereal/types/utility.hpp"
#include "cereal/types/memory.hpp"
#include "cereal/archives/binary.hpp"
#include "testthis.hpp"
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


int main() {
    google::dense_hash_map<long, float> V;
    std::vector<long> S;
    google::dense_hash_map<long, std::vector<std::pair<long, long>>> A;
    google::dense_hash_map<std::pair<long, long>, float, pair_hash> C;
    google::dense_hash_map<
        long,
        google::dense_hash_map<
            std::pair<long, long>,
            std::vector<std::pair<long, float>>,
            pair_hash
        >
    > P;

    // std::ifstream os("out.cereal", std::ios::binary);
    // cereal::BinaryInputArchive load(os);
    // load(V, S, A, C, P);

    // solve(V, S, A, C, P, 1000);
}

int CPP_get_normal_intersections(
        google::dense_hash_map<std::pair<long, long>, CPP_Intersection, pair_hash> &out,
        google::dense_hash_map<long, std::vector<long>> &successors,
        google::dense_hash_map<long, std::pair<float, float>> &data) {
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
        std::vector<std::pair<long, long>> edges) {
    for (unsigned long i = 0; i < edges.size(); ++i) {
        for (unsigned long j = 0; j < edges.size(); ++j) {
            if (i == j)
                continue;
            out.push_back(std::pair<std::pair<long, long>, std::pair<long, long>>(edges[i], edges[j]));
        }
    }
}


int CPP_make_low_angle_intersections_uncertain(
        google::dense_hash_map<long, std::vector<long>> &successors,
        google::dense_hash_map<long, std::pair<float, float>> &data) {

    // TODO
    
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
    std::ofstream os("out.cereal", std::ios::binary);
    cereal::BinaryOutputArchive archive(os);
    archive(V, S, A, C, P);

    //std::ifstream os("out.cereal", std::ios::binary);
    //cereal::BinaryInputArchive load(os);
    //load(V, S, A, C, P);

    google::dense_hash_map<long, google::dense_hash_map<std::pair<long, long>, float, pair_hash>> Q = {};
    Q.set_empty_key(0);
    for (auto &s : S) {
        Q[s] = {};
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
