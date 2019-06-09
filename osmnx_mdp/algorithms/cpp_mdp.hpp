#include <google/dense_hash_map>
#include <vector>
#include <set>

#include "../cpp_lib.hpp"


// TODO Rename CPP_* things
// C++ has camelcase style

struct CPP_Intersection {
    long left_node;
    long right_node;
    long straight_on_node;
    std::pair<long, long> origin_edge;
    bool operator==(CPP_Intersection other) const{
        if (
                left_node == other.left_node &&
                right_node == other.right_node &&
                straight_on_node == other.straight_on_node)
            return true;
        return false;
    }
};


class CPP_MDP {
    public:
        CPP_MDP();
        ~CPP_MDP();


        std::vector<long> *S;
        google::dense_hash_map<long, std::vector<std::pair<long, long>>> *A;
        google::dense_hash_map<std::pair<long, long>, float, pair_hash> *C;
        // It is needed that this is DOUBLE. Especially (B)RTDP
        // needs precision. Who knows how bad the effect in VI is.
        google::dense_hash_map<
            long,
            google::dense_hash_map<
                std::pair<long, long>,
                std::vector<std::pair<long, double>>,
                pair_hash
            >
        > *P;

        google::dense_hash_map<long, float> V;
        // TODO Only save the index of the action, not the whole action again.
        google::dense_hash_map<long, std::pair<long, long>> policy;

        long start;
        long goal;

        std::vector<long> angle_nodes;
        std::vector<CPP_Intersection> close_intersections;
        google::dense_hash_map<std::pair<long, long>, double, pair_hash> *edge_data;
        google::dense_hash_map<long, std::pair<double, double>> *node_data;
        google::dense_hash_map<long, std::vector<long>> *successors;

        std::set<long> uncertain_nodes;

        int init(
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
            google::dense_hash_map<long, std::vector<long>> *successors);
        int setup(long start, long goal);
        int make_goal_self_absorbing();

        // Make taking the action of following given edge probabilistic,
        // i.e. end up in the expected edge only 90% of the time and end
        // up in other_node 10% of the time.
        // If the edge is already probabilistic, decrease its chance (e.g.
        // from 90% to 80% and so on).
        // Modifies temp_P inplace.
        int make_edge_uncertain(
            google::dense_hash_map<
                std::pair<long, long>,
                std::vector<std::pair<long, double>>,
                pair_hash
            > &temp_P,
            const std::pair<long, long> &edge,
            const long &other_node);
        int get_normal_intersections(
                google::dense_hash_map<std::pair<long, long>, CPP_Intersection, pair_hash> &out);
        int make_close_intersections_uncertain(float max_length=100);
        int make_low_angle_intersections_uncertain(float max_angle=30);
        int solve(int max_iter=50000, double eps=1e-20);
        int get_policy();
        std::vector<long> drive(google::dense_hash_map<long, long> &diverge_policy);
};
