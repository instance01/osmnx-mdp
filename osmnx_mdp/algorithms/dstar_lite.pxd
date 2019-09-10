# cython: language_level=3
cimport osmnx_mdp.algorithms.algorithm
from osmnx_mdp.algorithms.mdp cimport MDP
from libcpp.unordered_map cimport unordered_map
from libcpp.string cimport string
from libcpp.pair cimport pair
from libcpp.vector cimport vector
from osmnx_mdp.algorithms.dense_hash_map cimport dense_hash_map

cdef extern from "cpp_dstar_lite.cpp":
    struct pair_hash:
        long long operator(pair[long, long] p)
    cppclass CPP_DStar_Lite "DStarLite":
        CPP_DStar_Lite()
        dense_hash_map[pair[long, long], double, pair_hash] rhs
        dense_hash_map[pair[long, long], double, pair_hash] g
        dense_hash_map[pair[long, long], pair[double, double], pair_hash] U

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
        int setup(long start, long goal, unordered_map[string, double] cfg)
        int compute_shortest_path()
        int drive(vector[long] &out, dense_hash_map[long, long] diverge_policy)

cdef class DStarLite(osmnx_mdp.algorithms.algorithm.Algorithm):
    cdef dense_hash_map[long, vector[long]] predecessors
    cdef dense_hash_map[long, vector[long]] successors
    cdef dense_hash_map[pair[long, long], double, pair_hash] cost
    cdef dense_hash_map[long, pair[double, double]] data

    cdef MDP mdp
    cdef CPP_DStar_Lite cpp

    cdef G
    cdef rhs
    cdef g
    cdef U
    cdef k

    cdef backup

    cdef long start
    cdef long goal

    cdef setup(self, long start, long goal, unordered_map[string, double] cfg)
    cdef compute_shortest_path(self)
    cdef solve(self)
    cdef drive(self, policy, diverge_policy)
