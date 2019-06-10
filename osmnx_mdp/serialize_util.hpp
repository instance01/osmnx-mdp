#ifndef SERIALIZE_UTIL_HEADER
#define SERIALIZE_UTIL_HEADER
#include <google/dense_hash_map>
#include "algorithms/cpp_mdp.hpp"
#include "algorithms/cpp_brtdp.hpp"

#include <fstream>
#include "algorithms/cereal/cereal.hpp"
#include "algorithms/cereal/types/concepts/pair_associative_container.hpp"
#include "algorithms/cereal/types/memory.hpp"
#include "algorithms/cereal/types/vector.hpp"
#include "algorithms/cereal/types/set.hpp"
#include "algorithms/cereal/types/utility.hpp"
#include "algorithms/cereal/archives/binary.hpp"

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
void serialize(Archive &ar, CPP_Intersection &intersection)
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
    google::dense_hash_map<std::pair<long, long>, float, pair_hash> C;
    google::dense_hash_map<
        long,
        google::dense_hash_map<
            std::pair<long, long>,
            std::vector<std::pair<long, double>>,
            pair_hash
        >
    > P;

    google::dense_hash_map<std::pair<long, long>, double, pair_hash> edge_data;
    google::dense_hash_map<long, std::pair<double, double>> node_data;
    google::dense_hash_map<long, std::vector<long>> successors;
};

struct SharedBRTDPData {
    std::vector<long> S;
    google::dense_hash_map<long, std::vector<std::pair<long, long>>> A;
    google::dense_hash_map<std::pair<long, long>, float, pair_hash> C;
    google::dense_hash_map<
        long,
        google::dense_hash_map<
            std::pair<long, long>,
            std::vector<std::pair<long, double>>,
            pair_hash
        >
    > P;

    google::dense_hash_map<long, std::pair<float, float>> data;
};

void save_mdp(CPP_MDP *mdp, std::string name);

void load_mdp(CPP_MDP *mdp, SharedMDPData &data, std::string name);

void save_brtdp(CPP_BRTDP *brtdp, std::string name);

void load_brtdp(CPP_BRTDP *brtdp, SharedBRTDPData &data, std::string name);

#endif
