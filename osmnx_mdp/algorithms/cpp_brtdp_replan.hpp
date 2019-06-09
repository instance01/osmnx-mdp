#include <google/dense_hash_map>
#include <random>
#include <vector>

#include "../cpp_lib.hpp"
#include "cpp_brtdp.hpp"


class CPP_BRTDP_REPLAN : public CPP_BRTDP {
    public:
        CPP_BRTDP_REPLAN();
        ~CPP_BRTDP_REPLAN();

        void replan(long curr_node);
        std::vector<long> get_path(
                google::dense_hash_map<long, long> diverge_policy,
                float beta=.02,
                bool always_replan=true);
};
