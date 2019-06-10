#include "catch/catch.hpp"
#include "../serialize_util.hpp"


TEST_CASE("Make sure run_trials works.", "[run_trials]") {
    CPP_BRTDP brtdp;
    SharedBRTDPData data;
    load_brtdp(&brtdp, data, "data/BRTDPrun_trials.cereal");

    brtdp.random_generator.seed(42069);
    brtdp.run_trials();

    CPP_BRTDP brtdp_want;
    SharedBRTDPData data_want;
    load_brtdp(&brtdp_want, data_want, "data/BRTDPrun_trialsWANT.cereal");

    REQUIRE(*(brtdp.S) == *(brtdp_want.S));
    REQUIRE(*(brtdp.A) == *(brtdp_want.A));
    REQUIRE(*(brtdp.C) == *(brtdp_want.C));
    REQUIRE(*(brtdp.P) == *(brtdp_want.P));
    REQUIRE(brtdp.vu == brtdp_want.vu);
    REQUIRE(brtdp.vl == brtdp_want.vl);
}

TEST_CASE("Make sure get_path works.", "[get_path]") {
    CPP_BRTDP brtdp;
    SharedBRTDPData data;
    load_brtdp(&brtdp, data, "data/BRTDPget_path.cereal");

    google::dense_hash_map<long, long> diverge_policy;
    diverge_policy.set_empty_key(0);
    std::vector<long> path = brtdp.get_path(diverge_policy);

    std::vector<long> path_want;
    {
        std::ifstream is("data/BRTDPget_pathWANT.cereal", std::ios::binary);
        cereal::BinaryInputArchive archive(is);
        archive(path_want);
    }

    REQUIRE(path == path_want);
}
