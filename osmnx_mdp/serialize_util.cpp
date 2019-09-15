#ifdef TESTS
#include "serialize_util.hpp"

void save_mdp(MDP *mdp, std::string name)
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

void load_mdp(MDP *mdp, SharedMDPData &data, std::string name)
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

void save_brtdp(BRTDP *brtdp, std::string name)
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

void load_brtdp(BRTDP *brtdp, SharedBRTDPData &data, std::string name)
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

void save_dstar(DStarLite *dstar, std::string name)
{
    std::ofstream os(name, std::ios::binary);
    cereal::BinaryOutputArchive archive(os);
    archive(
        dstar->rhs,
        dstar->g,
        dstar->U,
        dstar->k,
        dstar->start,
        dstar->goal,
        *(dstar->predecessors),
        *(dstar->successors),
        *(dstar->data),
        *(dstar->cost),
        dstar->nodes
    );
}

void load_dstar(DStarLite *dstar, SharedDStarLiteData &data, std::string name)
{
    std::ifstream is(name, std::ios::binary);
    cereal::BinaryInputArchive archive(is);
    archive(
        dstar->rhs,
        dstar->g,
        dstar->U,
        dstar->k,
        dstar->start,
        dstar->goal,
        data.predecessors,
        data.successors,
        data.data,
        data.cost,
        dstar->nodes
    );

    dstar->predecessors = &data.predecessors;
    dstar->successors = &data.successors;
    dstar->data = &data.data;
    dstar->cost = &data.cost;
}
#endif
