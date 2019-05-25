# cython: language_level=3
from osmnx_mdp.algorithms.algorithm cimport Algorithm
cimport osmnx_mdp.algorithms.algorithm

from libcpp.unordered_map cimport unordered_map

#cdef extern from "cpp_dstar_lite.cpp":
#    cdef cppclass cpp_DStar_Lite:
#        cpp_DStar_Lite() except +
#        unordered_map[long, float] rhs

cdef class DStar_Lite(osmnx_mdp.algorithms.algorithm.Algorithm):
    #cdef cpp_DStar_Lite cpp

    cdef G
    cdef rhs
    cdef g
    cdef U
    cdef k

    cdef backup

    cdef long start
    cdef long goal

    cdef setup(self, long start, long goal)
    cdef heuristic_func(self, node)
    cdef calculate_key(self, node)
    cdef update_vertex(self, u)
    cdef compute_shortest_path(self)
    cdef solve(self)
    cdef drive(self, policy, diverge_policy)
    #cdef fffa(self, policy, diverge_policy)
