# cython: language_level=3
cimport osmnx_mdp.algorithms.algorithm

from libcpp.pair cimport pair
from libcpp.vector cimport vector
from osmnx_mdp.algorithms.dense_hash_map cimport dense_hash_map

cdef extern from "cpp_dstar_lite.cpp":
    struct pair_hash:
        long long operator(pair[long, long] p)
    cppclass CPP_DStar_Lite "DStar_Lite":
        CPP_DStar_Lite()
        dense_hash_map[long, double] rhs
        dense_hash_map[long, double] g
        dense_hash_map[long, pair[double, double]] U

        int k
        long start
        long goal

        dense_hash_map[long, vector[long]] *predecessors
        dense_hash_map[long, vector[long]] *successors
        dense_hash_map[long, pair[double, double]] *data
        dense_hash_map[pair[long, long], double, pair_hash] *cost
        vector[long] nodes

        int init(
                dense_hash_map[long, vector[long]] *predecessors,
                dense_hash_map[long, vector[long]] *successors,
                dense_hash_map[pair[long, long], double, pair_hash] *cost,
                dense_hash_map[long, pair[double, double]] *data)
        int setup(long start, long goal)
        pair[double, double] calculate_key(long node)
        int update_vertex(long u)
        int compute_shortest_path()
        int drive(vector[long] &out, dense_hash_map[long, long] diverge_policy)

cdef class DStar_Lite(osmnx_mdp.algorithms.algorithm.Algorithm):
    cdef dense_hash_map[long, vector[long]] predecessors
    cdef dense_hash_map[long, vector[long]] successors
    cdef dense_hash_map[pair[long, long], double, pair_hash] cost
    cdef dense_hash_map[long, pair[double, double]] data

    cdef CPP_DStar_Lite cpp

    cdef G
    cdef rhs
    cdef g
    cdef U
    cdef k

    cdef backup

    cdef long start
    cdef long goal

    cdef setup(self, long start, long goal)
    cdef calculate_key(self, node)
    cdef update_vertex(self, u)
    cdef compute_shortest_path(self)
    cdef solve(self)
    cdef drive(self, policy, diverge_policy)
