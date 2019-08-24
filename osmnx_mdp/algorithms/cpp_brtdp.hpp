#ifndef CPP_BRTDP_HEADER
#define CPP_BRTDP_HEADER
#include <google/dense_hash_map>
#include <unordered_map>
#include <random>
#include <vector>

#include "../cpp_lib.hpp"


class BRTDP {
    public:
        BRTDP();
        virtual ~BRTDP();

        std::default_random_engine random_generator;

        double alpha;
        double tau;

        double heuristic_max_speed;

        long start;
        long goal;

        std::vector<long> *S;
        google::dense_hash_map<long, std::vector<std::pair<long, long>>> *A;
        google::dense_hash_map<std::pair<long, long>, double, pair_hash> *C;
        google::dense_hash_map<
            long,
            google::dense_hash_map<
                std::pair<long, long>,
                std::vector<std::pair<long, double>>,
                pair_hash
            >
        > *P;

        google::dense_hash_map<long, std::vector<long>> *predecessors;
        google::dense_hash_map<long, std::pair<double, double>> *data;

        // TODO: The names may make sense with the paper, but not for anyone else, rename.
        google::dense_hash_map<std::pair<long, long>, double, pair_hash> vu;
        google::dense_hash_map<std::pair<long, long>, double, pair_hash> vl;

        double get_Q_value(
            google::dense_hash_map<std::pair<long, long>, double, pair_hash> &v,
            const std::pair<long, long> &s,
            const std::pair<long, long> &a);
        std::pair<std::pair<long, long>, double> get_minimum_action(
            google::dense_hash_map<std::pair<long, long>, double, pair_hash> &v,
            const std::pair<long, long> &x);
        std::pair<long, long> update_v(
            google::dense_hash_map<std::pair<long, long>, double, pair_hash> &v,
            const std::pair<long, long> &x);
        int init(
            std::vector<long> *S,
            google::dense_hash_map<long, std::vector<std::pair<long, long>>> *A,
            google::dense_hash_map<std::pair<long, long>, double, pair_hash> *C,
            google::dense_hash_map<
                long,
                google::dense_hash_map<
                    std::pair<long, long>,
                    std::vector<std::pair<long, double>>,
                    pair_hash
                >
            > *P,
            google::dense_hash_map<long, std::vector<long>> *predecessors,
            google::dense_hash_map<long, std::pair<double, double>> *data);
        int setup(const long &start, const long &goal, std::unordered_map<std::string, double> cfg);
        int run_trial(const double &t);
        double get_outcome_distribution(
            const long pred,
            const std::pair<long, long> &curr_min_action,
            std::vector<double> &distribution);
        long select_node_probabilistically(
            const long pred,
            const std::pair<long, long> &curr_min_action,
            const std::vector<double> &distribution);
        int run_trials();
        std::vector<long> get_path(google::dense_hash_map<long, long> &diverge_policy);

        void init_upper_bound_heuristic();
        void init_lower_bound_heuristic();
};
#endif
