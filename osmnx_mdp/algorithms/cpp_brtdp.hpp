#include <google/dense_hash_map>
#include <random>
#include <vector>


// TODO: Copied from dstar lite
struct pair_hash {
    long long operator () (const std::pair<long, long> &p) const {
        long long ret = p.first;
        ret <<= 32;
        return ret + p.second;
    }
};


class CPP_BRTDP {
    public:
        CPP_BRTDP();
        ~CPP_BRTDP();

        long start;
        long goal;

        std::default_random_engine random_generator;

        // TODO: The names may make sense with the paper, but not for anyone else, rename.
        google::dense_hash_map<long, double> vu;
        google::dense_hash_map<long, double> vl;

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

        double get_Q_value(
                google::dense_hash_map<long, double> &v,
                const long &s,
                const std::pair<long, long> &a);
        std::pair<long, long> update(google::dense_hash_map<long, double> &v, const long &x);
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
        std::vector<long> get_path();
};
