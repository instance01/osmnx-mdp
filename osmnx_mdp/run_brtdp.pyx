# cython: language_level=3
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
    with open('data/munich.pickle', 'rb') as f:
        locs, G = pickle.load(f)

    start = locs[0]['start']
    goal = locs[0]['goal']

    remove_zero_cost_loops(G)
    remove_dead_ends(G, goal)

    mdp = MDP(G)
    mdp.setup(start, goal)

    cdef BRTDP brtdp
    brtdp = BRTDP(mdp)
    brtdp.setup(start, goal)
    brtdp.run_trials()

    vl = {}
    for k, v in brtdp.vl:
        vl[k] = v

    path = brtdp.get_path()

    draw_value_graph(mdp.G, path, vl)

run()
