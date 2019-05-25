# cython: language_level=3
import pickle

import matplotlib.pyplot as plt
import networkx as nx

from osmnx_mdp.algorithms.mdp cimport MDP
from osmnx_mdp.lib cimport get_time_to_drive
from osmnx_mdp.lib cimport get_node_properties


# IT IS OF VITAL IMPORTANCE TO SET THE TYPE TO MDP
# MDP mdp
cdef prepare_graph_for_drawing(MDP mdp, policy):
    G2 = nx.MultiDiGraph()
    for node in mdp.G.nodes.data():
        G2.add_node(node[0], pos=(node[1]['x'], node[1]['y']))

    for s in mdp.S:
        if s in policy:
            G2.add_edge(*mdp.A[s][policy[s]])

    G2.graph.update({
        'crs': mdp.G.graph['crs'],
        'name': mdp.G.graph['name']})

    return G2


#def draw_mdp():
#    with open('model6.pickle', 'rb') as f:
#        mdp, V, Q = pickle.load(f)
#
#    draw_mdp(mdp, V, Q)


# IT IS OF VITAL IMPORTANCE TO SET THE TYPE TO MDP
# MDP mdp
cdef draw_mdp(MDP mdp, V, Q):
    policy = mdp.get_policy(V)
    path = mdp.drive(policy, {})

    print(policy)
    print(path)
    print('Minutes spent driving this route:', get_time_to_drive(path, mdp.G))

    G2 = prepare_graph_for_drawing(mdp, policy)

    nc, ns = get_node_properties(G2, path, V, extra=mdp.close_nodes)
    nx.draw_networkx(
            G2,
            nx.get_node_attributes(G2, 'pos'),
            node_size=ns,
            node_color=nc,
            node_zorder=2,
            with_labels=False)

    # Draw all edges, not only best actions/edges
    edge_list = []
    for edge in mdp.G.edges():
        if edge not in G2.edges():
            edge_list.append(edge)
    nx.draw_networkx_edges(
            G2,
            nx.get_node_attributes(G2, 'pos'),
            edge_list,
            arrows=False)

    plt.show()


#from osmnx_mdp.algorithms.mdp cimport MDP
from osmnx_mdp.lib cimport remove_dead_ends
from osmnx_mdp.lib cimport remove_zero_cost_loops


cdef run():
    with open('data/munich.pickle', 'rb') as f:
        locs, G = pickle.load(f)

    start = locs[0]['start']
    goal = locs[0]['goal']

    remove_zero_cost_loops(G)
    remove_dead_ends(G, goal)

    mdp = MDP(G)
    print(dir(mdp))
    mdp.setup(start, goal)
    # TODO: Lol max_iter is not getting passed to the C++ function..
    V, _ = mdp.solve_value_iteration(gamma=1., max_iter=10000)

    draw_mdp(mdp, V, {})
