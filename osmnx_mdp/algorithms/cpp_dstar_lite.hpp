#ifndef CPP_DSTAR_LITE_HEADER
#define CPP_DSTAR_LITE_HEADER
#include <google/dense_hash_map>
#include <unordered_map>
#include <vector>
#include <queue>

#include "../cpp_lib.hpp"


class DStar_Lite {
    private:
        std::pair<long, double> get_min_successor(const long &node);

    public:
        DStar_Lite();
        ~DStar_Lite();

        google::dense_hash_map<long, double> rhs;
        google::dense_hash_map<long, double> g;
        google::dense_hash_map<long, std::pair<double, double>> U;

        google::dense_hash_map<long, double> heuristic_map;

        int k = 0;
        long start;
        long goal;

        google::dense_hash_map<long, std::vector<long>> *predecessors;
        google::dense_hash_map<long, std::vector<long>> *successors;
        google::dense_hash_map<long, std::pair<double, double>> *data;
        google::dense_hash_map<std::pair<long, long>, double, pair_hash> *cost;
        std::vector<long> nodes;

        int init(
                google::dense_hash_map<long, std::vector<long>> *predecessors,
                google::dense_hash_map<long, std::vector<long>> *successors,
                google::dense_hash_map<std::pair<long, long>, double, pair_hash> *cost,
                google::dense_hash_map<long, std::pair<double, double>> *data);
        int setup(const long &start, const long &goal, std::unordered_map<std::string, double> cfg);
        std::pair<double, double> calculate_key(const long &node);
        int update_vertex(const long &node);
        int compute_shortest_path();
        int drive(std::vector<long> &out, google::dense_hash_map<long, long> &diverge_policy);

        void init_heuristic();
};
#endif
