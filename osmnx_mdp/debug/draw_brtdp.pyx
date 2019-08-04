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
    with open('../data/munich.pickle', 'rb') as f:
        locs, G = pickle.load(f)

    with open('../simulation.pickle', 'rb') as f:
        results = pickle.load(f)

    start = locs[0]['start']
    goal = locs[0]['goal']

    remove_zero_cost_loops(G)
    remove_dead_ends(G, goal)

    for result in results[0]:
        # algorithm: DStar_Lite, BRTDP, BRTDP_REPLAN, MDP
        # id: StarnbergLONG, BERLINLONG1, BERLINLONG, AURIMUN
        #if result['algorithm'] == 'BRTDP' and result['id'] == 'AURIMUN':
        #if result['algorithm'] == 'BRTDP' and result['id'].startswith('BERLIN'):
        if result['algorithm'] == 'DStar_Lite':
            path = result['path']
            diverge = result['diverge_policy']

            draw_value_graph(G, path, None, extra=diverge)
            break


run()
