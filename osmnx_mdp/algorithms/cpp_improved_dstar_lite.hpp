#ifndef CPP_IMPRDSTAR_LITE_HEADER
#define CPP_IMPRDSTAR_LITE_HEADER
#include <google/dense_hash_map>
#include <unordered_map>
#include <vector>
#include <queue>

#include "../cpp_lib.hpp"
#include "cpp_dstar_lite.hpp"


class ImprovedDStarLite : public DStarLite {
    public:
        ImprovedDStarLite();
        ~ImprovedDStarLite();

        google::dense_hash_map<long, google::dense_hash_map<std::pair<long, long>, double, pair_hash>> *num_outcomes;

        int init(
                google::dense_hash_map<long, std::vector<long>> *predecessors,
                google::dense_hash_map<long, std::vector<long>> *successors,
                google::dense_hash_map<std::pair<long, long>, double, pair_hash> *cost,
                google::dense_hash_map<long, std::pair<double, double>> *data,
                google::dense_hash_map<long, google::dense_hash_map<std::pair<long, long>, double, pair_hash>> *num_outcomes);

        double get_penalty(long predecessor, std::pair<long, long> action) override;
};
#endif
