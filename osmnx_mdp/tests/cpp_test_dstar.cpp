#include "catch/catch.hpp"
#include "../serialize_util.hpp"


TEST_CASE("Make sure compute_shortest_path works.", "[compute_shortest_path]") {
    DStar_Lite dstar;
    SharedDStarLiteData data;
    load_dstar(&dstar, data, "data/DSTARcompute_shortest_path.cereal");

    dstar.U.set_deleted_key(1);

    dstar.compute_shortest_path();

    DStar_Lite dstar_want;
    SharedDStarLiteData data_want;
    load_dstar(&dstar_want, data_want, "data/DSTARcompute_shortest_pathWANT.cereal");

    REQUIRE(dstar.rhs == dstar_want.rhs);
    REQUIRE(dstar.g == dstar_want.g);
    REQUIRE(dstar.U == dstar_want.U);
    REQUIRE(dstar.k == dstar_want.k);
}

TEST_CASE("Make sure get_path works.", "[get_path]") {
    DStar_Lite dstar;
    SharedDStarLiteData data;
    load_dstar(&dstar, data, "data/DSTARdrive.cereal");

    dstar.U.set_deleted_key(1);

    google::dense_hash_map<long, long> diverge_policy;
    diverge_policy.set_empty_key(0);
    std::vector<long> path;
    dstar.drive(path, diverge_policy);

    std::vector<long> path_want;
    {
        std::ifstream is("data/DSTARdriveWANT.cereal", std::ios::binary);
        cereal::BinaryInputArchive archive(is);
        archive(path_want);
    }

    REQUIRE(path == path_want);
}
