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

// using spp::google::dense_hash_map;

int Xd(float gamma, int state) {
    return 1;
}

// void cpp_get_Q_value(
//         google::dense_hash_map<std::pair<long, long>, float> C,
//         google::dense_hash_map<std::pair<long, long>, float> P,
//         google::dense_hash_map<int, int> prev_V,
//         float gamma,
//         long state,
//         std::pair<long, long> action) {
// }

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

    return 0;
}
