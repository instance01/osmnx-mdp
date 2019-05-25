# cython: language_level=3
from osmnx_mdp.algorithms.mdp cimport MDP

cdef prepare_graph_for_drawing(MDP mdp, policy)
cdef draw_mdp(MDP mdp, V, Q)
cdef run()
