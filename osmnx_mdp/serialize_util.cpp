#include "serialize_util.hpp"

void save_mdp(CPP_MDP *mdp, std::string name)
{
    std::ofstream os(name, std::ios::binary);
    cereal::BinaryOutputArchive archive(os);
    archive(
        *(mdp->S),
        *(mdp->A),
        *(mdp->C),
        *(mdp->P),
        mdp->V,
        mdp->policy,
        mdp->start,
        mdp->goal,
        mdp->angle_nodes,
        mdp->close_intersections,
        *(mdp->edge_data),
        *(mdp->node_data),
        *(mdp->successors),
        mdp->uncertain_nodes
    );
}

void load_mdp(CPP_MDP *mdp, SharedMDPData &data, std::string name)
{
    std::ifstream is(name, std::ios::binary);
    cereal::BinaryInputArchive archive(is);
    archive(
        data.S,
        data.A,
        data.C,
        data.P,
        mdp->V,
        mdp->policy,
        mdp->start,
        mdp->goal,
        mdp->angle_nodes,
        mdp->close_intersections,
        data.edge_data,
        data.node_data,
        data.successors,
        mdp->uncertain_nodes
    );

    mdp->S = &data.S;
    mdp->A = &data.A;
    mdp->C = &data.C;
    mdp->P = &data.P;

    mdp->edge_data = &data.edge_data;
    mdp->node_data = &data.node_data;
    mdp->successors = &data.successors;
}

void save_brtdp(CPP_BRTDP *brtdp, std::string name)
{
    std::ofstream os(name, std::ios::binary);
    cereal::BinaryOutputArchive archive(os);
    archive(
        *(brtdp->S),
        *(brtdp->A),
        *(brtdp->C),
        *(brtdp->P),
        brtdp->vl,
        brtdp->vu,
        brtdp->start,
        brtdp->goal,
        *(brtdp->data)
    );
}

void load_brtdp(CPP_BRTDP *brtdp, SharedBRTDPData &data, std::string name)
{
    std::ifstream is(name, std::ios::binary);
    cereal::BinaryInputArchive archive(is);
    archive(
        data.S,
        data.A,
        data.C,
        data.P,
        brtdp->vl,
        brtdp->vu,
        brtdp->start,
        brtdp->goal,
        data.data
    );

    brtdp->S = &data.S;
    brtdp->A = &data.A;
    brtdp->C = &data.C;
    brtdp->P = &data.P;

    brtdp->data = &data.data;
}
