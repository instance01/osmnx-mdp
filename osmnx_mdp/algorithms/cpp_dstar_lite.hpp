#include <google/dense_hash_map>
#include <vector>

#include "../cpp_lib.hpp"


class cpp_DStar_Lite {
    private:
        // For a given node, return the successor which lies on the minimum sum
        // of the estimated path to the node and the edge cost between the node
        // and the successor.
        std::pair<long, float> get_min_successor(long node);

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
                google::dense_hash_map<long, std::vector<long>> *predecessors,
                google::dense_hash_map<long, std::vector<long>> *successors,
                google::dense_hash_map<std::pair<long, long>, float, pair_hash> *cost,
                google::dense_hash_map<long, std::pair<float, float>> *data);
        int setup(long start, long goal);
        float heuristic(long node);
        std::pair<float, float> calculate_key(long node);
        int update_vertex(long u);
        int compute_shortest_path();
        int drive(std::vector<long> &out, google::dense_hash_map<long, long> &diverge_policy);
};
