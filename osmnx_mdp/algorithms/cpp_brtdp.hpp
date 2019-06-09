#ifndef CPP_BRTDP_HEADER
#define CPP_BRTDP_HEADER
#include <google/dense_hash_map>
#include <random>
#include <vector>

#include "../cpp_lib.hpp"


class CPP_BRTDP {
    public:
        CPP_BRTDP();
        virtual ~CPP_BRTDP();

        long start;
        long goal;

        std::default_random_engine random_generator;

        // TODO: The names may make sense with the paper, but not for anyone else, rename.
        google::dense_hash_map<long, double> vu;
        google::dense_hash_map<long, double> vl;

        std::vector<long> *S;
        google::dense_hash_map<long, std::vector<std::pair<long, long>>> *A;
        google::dense_hash_map<std::pair<long, long>, float, pair_hash> *C;
        // TODO It is needed that this is DOUBLE. Especially (B)RTDP
        // needs precision. Who knows how bad the effect in VI is.
        google::dense_hash_map<
            long,
            google::dense_hash_map<
                std::pair<long, long>,
                std::vector<std::pair<long, double>>,
                pair_hash
            >
        > *P;
        google::dense_hash_map<long, std::pair<float, float>> *data;

        double get_Q_value(
                google::dense_hash_map<long, double> &v,
                const long &s,
                const std::pair<long, long> &a);
        std::pair<std::pair<long, long>, double> get_minimum_action(
                google::dense_hash_map<long, double> &v,
                const long &x);
        std::pair<long, long> update_v(google::dense_hash_map<long, double> &v, const long &x);
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
        int setup(long start, long goal);
        int run_trial(double t);
        int run_trials(double alpha=1e-10, double t=10);
        std::vector<long> get_path(google::dense_hash_map<long, long> diverge_policy);
};
#endif
