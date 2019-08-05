# cython: language_level=3
import matplotlib.pyplot as plt
import pickle
import networkx as nx

from osmnx_mdp.lib cimport get_node_properties
from osmnx_mdp.lib cimport remove_zero_cost_loops
from osmnx_mdp.lib cimport remove_dead_ends

from osmnx_mdp.lib cimport draw_value_graph

from osmnx_mdp.algorithms.dstar_lite cimport DStar_Lite

cdef run():
    with open('data/berlin.pickle', 'rb') as f:
        locs, G = pickle.load(f)

    start = locs[2]['start']
    goal = locs[2]['goal']

    remove_zero_cost_loops(G)
    remove_dead_ends(G, goal)

    cfg = {}
    dstar = DStar_Lite(G)
    dstar.setup(start, goal, cfg)
    dstar.compute_shortest_path()
    col_nodes = list(dstar.drive({}, {}))
    print(col_nodes)

    #nc, ns = get_node_properties(dstar.G, col_nodes)
    #for node in dstar.G.nodes(data=True):
    #    dstar.G.node[node[0]]['pos'] = (node[1]['x'], node[1]['y'])
    #nx.draw_networkx(
    #        dstar.G,
    #        nx.get_node_attributes(dstar.G, 'pos'),
    #        node_size=ns,
    #        node_color=nc,
    #        node_zorder=2,
    #        with_labels=False)

    draw_value_graph(G, col_nodes)

    #plt.show()


run()
