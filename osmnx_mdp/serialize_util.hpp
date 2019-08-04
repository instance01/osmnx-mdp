#ifndef SERIALIZE_UTIL_HEADER
#define SERIALIZE_UTIL_HEADER
#include <google/dense_hash_map>
#include "algorithms/cpp_mdp.hpp"
#include "algorithms/cpp_brtdp.hpp"
#include "algorithms/cpp_dstar_lite.hpp"

#include <fstream>
#include "external/cereal/cereal.hpp"
#include "external/cereal/types/concepts/pair_associative_container.hpp"
#include "external/cereal/types/memory.hpp"
#include "external/cereal/types/vector.hpp"
#include "external/cereal/types/set.hpp"
#include "external/cereal/types/unordered_set.hpp"
#include "external/cereal/types/utility.hpp"
#include "external/cereal/archives/binary.hpp"

#include <iostream>

namespace google {
    template <class Archive, typename... Args>
    void save(Archive &ar, google::dense_hash_map<Args...> const &map)
    {
        ar(cereal::make_size_tag(static_cast<cereal::size_type>(map.size())));

        for(const auto &i : map)
            ar(cereal::make_map_item(i.first, i.second));
    }

    template <class Archive, typename... Args>
    void load(Archive &ar, google::dense_hash_map<Args...> &map)
    {
        typename google::dense_hash_map<Args...>::key_type key;
        map.set_empty_key(key);

        cereal::size_type size;
        ar(cereal::make_size_tag(size));

        map.clear();

        auto hint = map.begin();
        for(size_t i = 0; i < size; ++i)
        {
            typename google::dense_hash_map<Args...>::key_type key;
            typename google::dense_hash_map<Args...>::mapped_type value;

            ar(cereal::make_map_item(key, value));
            hint = map.insert(hint, std::make_pair(std::move(key), std::move(value)));
        }
    }
}

template <class Archive>
void serialize(Archive &ar, Intersection &intersection)
{
    ar(
        intersection.left_node,
        intersection.right_node,
        intersection.straight_on_node,
        intersection.origin_edge
    );
}

struct SharedMDPData {
    std::vector<long> S;
    google::dense_hash_map<long, std::vector<std::pair<long, long>>> A;
    google::dense_hash_map<std::pair<long, long>, double, pair_hash> C;
    google::dense_hash_map<
        std::pair<long, long>,
        std::vector<std::pair<long, double>>,
        pair_hash
    > P;

    google::dense_hash_map<std::pair<long, long>, double, pair_hash> edge_data;
    google::dense_hash_map<long, std::pair<double, double>> node_data;
    google::dense_hash_map<long, std::vector<long>> successors;
};

struct SharedBRTDPData {
    std::vector<long> S;
    google::dense_hash_map<long, std::vector<std::pair<long, long>>> A;
    google::dense_hash_map<std::pair<long, long>, double, pair_hash> C;
    google::dense_hash_map<
        std::pair<long, long>,
        std::vector<std::pair<long, double>>,
        pair_hash
    > P;

    google::dense_hash_map<long, std::pair<double, double>> data;
};

struct SharedDStarLiteData {
    google::dense_hash_map<long, std::vector<long>> predecessors;
    google::dense_hash_map<long, std::vector<long>> successors;
    google::dense_hash_map<long, std::pair<double, double>> data;
    google::dense_hash_map<std::pair<long, long>, double, pair_hash> cost;
};

void save_mdp(MDP *mdp, std::string name);

void load_mdp(MDP *mdp, SharedMDPData &data, std::string name);

void save_brtdp(BRTDP *brtdp, std::string name);

void load_brtdp(BRTDP *brtdp, SharedBRTDPData &data, std::string name);

void save_dstar(DStar_Lite *brtdp, std::string name);

void load_dstar(DStar_Lite *brtdp, SharedDStarLiteData &data, std::string name);

#endif
