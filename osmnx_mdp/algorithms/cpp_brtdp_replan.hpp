#include <google/dense_hash_map>
#include <random>
#include <vector>

#include "../cpp_lib.hpp"
#include "cpp_brtdp.hpp"


class BRTDP_REPLAN : public BRTDP {
    public:
        BRTDP_REPLAN();
        ~BRTDP_REPLAN();

        double beta;
        bool always_replan;

        int setup(const long &start, const long &goal, std::unordered_map<std::string, double> cfg);
        void replan(const long &curr_node);
        std::vector<long> get_path(google::dense_hash_map<long, long> &diverge_policy);
};
