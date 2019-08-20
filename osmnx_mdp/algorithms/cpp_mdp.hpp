#ifndef CPP_MDP_HEADER
#define CPP_MDP_HEADER
#include <google/dense_hash_map>
#include <unordered_map>
#include <vector>
#include <set>
#include <unordered_set>

#include "../cpp_lib.hpp"


// TODO Rename CPP_* things
// C++ has camelcase style

struct Intersection {
    long left_node;
    long right_node;
    long straight_on_node;
    std::pair<long, long> origin_edge;
    bool operator==(Intersection other) const{
        return left_node == other.left_node &&
            right_node == other.right_node &&
            straight_on_node == other.straight_on_node;
    };
};

struct intersection_hash {
    // Same as pair_hash from cpp_lib.hpp
    long long operator () (const Intersection &intersection) const {
        long long ret = intersection.origin_edge.first;
        ret <<= 32;
        return ret + intersection.origin_edge.second;
    }
};

class MDP {
    public:
        MDP();
        ~MDP();

        std::vector<long> *S;
        google::dense_hash_map<long, std::vector<std::pair<long, long>>> *A;
        google::dense_hash_map<std::pair<long, long>, double, pair_hash> *C;
        // It is needed that this is DOUBLE. Especially (B)RTDP
        // needs precision. Who knows how bad the effect in VI is.
        google::dense_hash_map<
            std::pair<long, long>,
            std::vector<std::pair<long, double>>,
            pair_hash
        > *P;

        google::dense_hash_map<std::pair<long, long>, double, pair_hash> V;
        // TODO Only save the index of the action, not the whole action again.
        google::dense_hash_map<std::pair<long, long>, std::pair<long, long>, pair_hash> policy;

        long start;
        long goal;

        double edge_uncertainty;

        std::vector<long> angle_nodes;
        std::unordered_set<Intersection, intersection_hash> close_intersections;
        google::dense_hash_map<std::pair<long, long>, double, pair_hash> *edge_data;
        google::dense_hash_map<std::pair<long, long>, std::pair<double, double>, pair_hash> *angle_data;
        google::dense_hash_map<long, std::pair<double, double>> *node_data;
        google::dense_hash_map<long, std::vector<long>> *predecessors;
        google::dense_hash_map<long, std::vector<long>> *successors;

        std::set<long> uncertain_nodes;

        int init(
            std::vector<long> *S,
            google::dense_hash_map<long, std::vector<std::pair<long, long>>> *A,
            google::dense_hash_map<std::pair<long, long>, double, pair_hash> *C,
            google::dense_hash_map<
                std::pair<long, long>,
                std::vector<std::pair<long, double>>,
                pair_hash
            > *P,
            google::dense_hash_map<std::pair<long, long>, double, pair_hash> *edge_data,
            google::dense_hash_map<std::pair<long, long>, std::pair<double, double>, pair_hash> *angle_data,
            google::dense_hash_map<long, std::pair<double, double>> *node_data,
            google::dense_hash_map<long, std::vector<long>> *successors,
            google::dense_hash_map<long, std::vector<long>> *predecessors);
        int setup(const long &start, const long &goal, std::unordered_map<std::string, double> cfg);
        int make_goal_self_absorbing();

        int make_edge_uncertain(
            google::dense_hash_map<
                std::pair<long, long>,
                std::vector<std::pair<long, double>>,
                pair_hash
            > &temp_P,
            const std::pair<long, long> &edge,
            const long &other_node,
            const double &uncertainty=.1);
        int make_intersection_uncertain(const Intersection &intersection, const long &intersection_node);
        int get_normal_intersections(
            google::dense_hash_map<std::pair<long, long>, Intersection, pair_hash> &out);
        int make_close_intersections_uncertain(const double &max_length=100);
        int make_low_angle_intersections_uncertain(const double &max_angle=30);
        double get_Q_value(
            google::dense_hash_map<std::pair<long, long>, double, pair_hash> &prev_V,
            const std::pair<long, long> &s_pair,
            const std::pair<long, long> &a);
        auto init_Q();
        bool converged(
                google::dense_hash_map<std::pair<long, long>, double, pair_hash> &prev_V,
                const double &eps);
        int solve(const int &max_iter=50000, const double &eps=1e-20);
        int get_policy();
        std::vector<long> drive(google::dense_hash_map<long, long> &diverge_policy);
};
#endif
