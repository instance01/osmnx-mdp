#include <google/dense_hash_map>
#include <set>
#include <vector>

// TODO: Copied from testthis.cpp
struct pair_hash {
    long long operator () (const std::pair<long, long> &p) const {
        long long ret = p.first;
        ret <<= 32;
        return ret + p.second;
    }
};

class cpp_DStar_Lite {
    public:
        cpp_DStar_Lite();
        ~cpp_DStar_Lite();
        google::dense_hash_map<long, float> rhs;
        google::dense_hash_map<long, float> g;
        google::dense_hash_map<long, std::pair<float, float>> U;
        int k = 0;
        long start;
        long goal;
        google::dense_hash_map<long, std::vector<long>> *predecessors;
        google::dense_hash_map<long, std::vector<long>> *successors;
        google::dense_hash_map<long, std::pair<float, float>> *data;
        google::dense_hash_map<std::pair<long, long>, float, pair_hash> *cost;
        std::vector<long> nodes;

        int init(
                google::dense_hash_map<std::pair<long, long>, float, pair_hash> *cost,
                google::dense_hash_map<long, std::pair<float, float>> *data,
                google::dense_hash_map<long, std::vector<long>> *predecessors,
                google::dense_hash_map<long, std::vector<long>> *successors);
        int setup(long start, long goal);
        float heuristic_func(long node);
        std::pair<float, float> calculate_key(long node);
        int update_vertex(long u);
        int compute_shortest_path();
        int drive(std::set<long> &visited, google::dense_hash_map<long, long> diverge_policy);
};
