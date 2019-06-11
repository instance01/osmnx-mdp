# cython: language_level=3
import numpy as np
import matplotlib.pyplot as plt
import pickle
import networkx as nx

from osmnx_mdp.lib cimport get_node_properties
from osmnx_mdp.lib cimport remove_zero_cost_loops
from osmnx_mdp.lib cimport remove_dead_ends
from osmnx_mdp.lib cimport draw_value_graph

from osmnx_mdp.algorithms.mdp cimport MDP
from osmnx_mdp.algorithms.brtdp cimport BRTDP

cdef run():
    with open('data/berlin.pickle', 'rb') as f:
        locs, G = pickle.load(f)

    with open('simulation.pickle', 'rb') as f:
        results = pickle.load(f)

    start = locs[0]['start']
    goal = locs[0]['goal']

    remove_zero_cost_loops(G)
    remove_dead_ends(G, goal)

    for result in results[0]:
        # DStar_Lite, MDP, BRTDP
        #if result['algorithm'] == 'BRTDP' and result['id'] == 'StarnbergLONG':
        if result['algorithm'] == 'BRTDP_REPLAN' and result['id'] == 'BERLINLONG1':
        #if result['algorithm'] == 'BRTDP' and result['id'].startswith('BERLIN'):
            path = result['path']
            diverge = list(result['diverge_policy'].keys())
            #x = [1585077893, 1585077917, 309569643, 325171173, 2542984712 0.153339, 304736098, 1803565650, 1878298424, 1585077862, ]
            #x = [1803565650, 1878298424, 1585077862, 1703346917, 1704104224, 1704104208, 1783231019]
            #x = [1703346917, 1704104224, 1704104208, 1783231019]
            draw_value_graph(G, path, None, extra=diverge)
            #draw_value_graph(G, path, None, extra=x)
            break

run()
