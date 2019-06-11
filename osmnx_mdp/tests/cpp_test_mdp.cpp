#include "catch/catch.hpp"
#include "../serialize_util.hpp"


// template<typename... Args>
// bool maps_equal(google::dense_hash_map<Args...> *map1, google::dense_hash_map<Args...> *map2) {
//     return map1->size() == map2->size() && std::equal(map1->begin(), map1->end(), map2->begin());
// }
// 
// template<typename... Args>
// bool vectors_equal(std::vector<Args...> *map1, std::vector<Args...> *map2) {
//     return map1->size() == map2->size() && std::equal(map1->begin(), map1->end(), map2->begin());
// }

TEST_CASE("Make sure goal becomes self absorbing", "[make_goal_self_absorbing]") {
    MDP mdp;
    SharedMDPData data;
    load_mdp(&mdp, data, "data/MDPmake_goal_self_absorbing.cereal");

    mdp.make_goal_self_absorbing();

    MDP mdp_want;
    SharedMDPData data_want;
    load_mdp(&mdp_want, data_want, "data/MDPmake_goal_self_absorbingWANT.cereal");

    REQUIRE(*(mdp.S) == *(mdp_want.S));
    REQUIRE(*(mdp.A) == *(mdp_want.A));
    REQUIRE(*(mdp.C) == *(mdp_want.C));
    REQUIRE(*(mdp.P) == *(mdp_want.P));
}

TEST_CASE("Make sure we generate correct normal intersections", "[get_normal_intersections]") {
    MDP mdp;
    SharedMDPData data;
    load_mdp(&mdp, data, "data/MDPmake_close_intersections_uncertain.cereal");

    google::dense_hash_map<std::pair<long, long>, Intersection, pair_hash> intersections;
    mdp.get_normal_intersections(intersections);

    google::dense_hash_map<std::pair<long, long>, Intersection, pair_hash> intersections_want;
    {
        std::ifstream is("data/MDPget_normal_intersectionsWANT.cereal", std::ios::binary);
        cereal::BinaryInputArchive archive(is);
        archive(intersections_want);
    }

    for (auto &x : intersections_want) {
        if (!(x.second == intersections[x.first])) {
            std::cout << "FUCK" << std::endl;
            std::cout << x.second.left_node << " " << x.second.right_node << std::endl;
            std::cout << intersections[x.first].left_node << " " << intersections[x.first].right_node << std::endl;
        }
        //std::cout << x.second.left_node << " " << intersections_want[x.first].left_node << std::endl;
    }

    REQUIRE(intersections == intersections_want);
}

TEST_CASE("Make sure make_close_intersections_uncertain works.", "[make_close_intersections_uncertain]") {
    MDP mdp;
    SharedMDPData data;
    load_mdp(&mdp, data, "data/MDPmake_close_intersections_uncertain.cereal");

    mdp.make_close_intersections_uncertain();

    MDP mdp_want;
    SharedMDPData data_want;
    load_mdp(&mdp_want, data_want, "data/MDPmake_close_intersections_uncertainWANT.cereal");

    REQUIRE(*(mdp.S) == *(mdp_want.S));
    REQUIRE(*(mdp.A) == *(mdp_want.A));
    REQUIRE(*(mdp.C) == *(mdp_want.C));
    REQUIRE(*(mdp.P) == *(mdp_want.P));
    REQUIRE(mdp.close_intersections == mdp_want.close_intersections);
}

TEST_CASE("Make sure make_low_angle_intersections_uncertain works.", "[make_low_angle_intersections_uncertain]") {
    MDP mdp;
    SharedMDPData data;
    load_mdp(&mdp, data, "data/MDPmake_low_angle_intersections_uncertain.cereal");

    mdp.make_low_angle_intersections_uncertain();

    MDP mdp_want;
    SharedMDPData data_want;
    load_mdp(&mdp_want, data_want, "data/MDPmake_low_angle_intersections_uncertainWANT.cereal");

    REQUIRE(*(mdp.S) == *(mdp_want.S));
    REQUIRE(*(mdp.A) == *(mdp_want.A));
    REQUIRE(*(mdp.C) == *(mdp_want.C));
    REQUIRE(*(mdp.P) == *(mdp_want.P));
    std::sort(mdp.angle_nodes.begin(), mdp.angle_nodes.end());
    std::sort(mdp_want.angle_nodes.begin(), mdp_want.angle_nodes.end());
    REQUIRE(mdp.angle_nodes == mdp_want.angle_nodes);
}

TEST_CASE("Make sure solving with value iteration works.", "[solve]") {
    MDP mdp;
    SharedMDPData data;
    load_mdp(&mdp, data, "data/MDPsolve.cereal");

    std::cout << "Running long test (solving MDP).." << std::endl;
    mdp.solve();
    std::cout << "Done." << std::endl;

    MDP mdp_want;
    SharedMDPData data_want;
    load_mdp(&mdp_want, data_want, "data/MDPsolveWANT.cereal");

    REQUIRE(*(mdp.S) == *(mdp_want.S));
    REQUIRE(*(mdp.A) == *(mdp_want.A));
    REQUIRE(*(mdp.C) == *(mdp_want.C));
    REQUIRE(*(mdp.P) == *(mdp_want.P));

    // for (auto &x : mdp.V) {
    //     if (x.second != mdp_want.V[x.first])
    //         std::cout << x.second << " " << mdp_want.V[x.first] << std::endl;
    // }

    REQUIRE(mdp.V == mdp_want.V);
}

TEST_CASE("Make sure driving is correct.", "[drive]") {
    MDP mdp;
    SharedMDPData data;
    load_mdp(&mdp, data, "data/MDPdrive.cereal");

    google::dense_hash_map<long, long> diverge_policy;
    diverge_policy.set_empty_key(0);
    std::vector<long> have = mdp.drive(diverge_policy);

    std::vector<long> want;
    {
        std::ifstream is("data/MDPdriveWANT.cereal", std::ios::binary);
        cereal::BinaryInputArchive archive(is);
        archive(want);
    }

    REQUIRE(have == want);
}
