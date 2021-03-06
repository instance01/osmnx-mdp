# cython: language_level=3
from libcpp.unordered_map cimport unordered_map
from libcpp.string cimport string
from libcpp.vector cimport vector
from libcpp.pair cimport pair
from osmnx_mdp.algorithms.dense_hash_map cimport dense_hash_map
from osmnx_mdp.algorithms.mdp cimport MDP
cimport osmnx_mdp.algorithms.algorithm
from osmnx_mdp.algorithms.brtdp cimport CPP_BRTDP


cdef extern from "cpp_brtdp_replan.hpp":
    struct pair_hash:
        long operator(pair[long, long])

    cppclass CPP_BRTDP_REPLAN "BRTDP_REPLAN"(CPP_BRTDP):
        CPP_BRTDP_REPLAN()

        vector[long] get_path(
                dense_hash_map[long, long] diverge_policy,
                double beta,
                bint always_replan)


cdef class BRTDP_REPLAN(osmnx_mdp.algorithms.algorithm.Algorithm):
    cdef dense_hash_map[long, vector[long]] predecessors
    cdef dense_hash_map[long, pair[double, double]] data
    cdef dense_hash_map[pair[long, long], double, pair_hash] vl

    cdef MDP mdp
    cdef CPP_BRTDP_REPLAN cpp

    cdef setup(self, long start, long goal, unordered_map[string, double] cfg)
    cdef run_trials(self)
    cdef get_path(self, diverge_policy)
    cdef solve(self)
    cdef drive(self, policy, diverge_policy)
