#ifndef CPP_BRTDP_HEADER
#define CPP_BRTDP_HEADER
#include <google/dense_hash_map>
#include <random>
#include <vector>

#include "../cpp_lib.hpp"


class BRTDP {
    public:
        BRTDP();
        virtual ~BRTDP();

        std::default_random_engine random_generator;

        long start;
        long goal;

        std::vector<long> *S;
        google::dense_hash_map<long, std::vector<std::pair<long, long>>> *A;
        google::dense_hash_map<std::pair<long, long>, float, pair_hash> *C;
        google::dense_hash_map<
            long,
            google::dense_hash_map<
                std::pair<long, long>,
                std::vector<std::pair<long, double>>,
                pair_hash
            >
        > *P;
        google::dense_hash_map<long, std::pair<float, float>> *data;

        // TODO: The names may make sense with the paper, but not for anyone else, rename.
        google::dense_hash_map<long, double> vu;
        google::dense_hash_map<long, double> vl;

        double get_Q_value(
            google::dense_hash_map<long, double> &v,
            const long &s,
            const std::pair<long, long> &a);
        std::pair<std::pair<long, long>, double> get_minimum_action(
            google::dense_hash_map<long, double> &v,
            const long &x);
        std::pair<long, long> update_v(
            google::dense_hash_map<long, double> &v,
            const long &x);
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
            google::dense_hash_map<long, std::pair<float, float>> *data);
        int setup(const long &start, const long &goal);
        int run_trial(const double &t);
        double get_outcome_distribution(
            const std::pair<long, long> &curr_min_action,
            std::vector<float> &distribution);
        long select_node_probabilistically(
            const std::pair<long, long> &curr_min_action,
            const std::vector<float> &distribution);
        int run_trials(const double &alpha=1e-10, const double &t=10);
        std::vector<long> get_path(google::dense_hash_map<long, long> &diverge_policy);
};
#endif
