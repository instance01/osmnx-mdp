# cython: language_level=3
from libcpp.vector cimport vector
from libcpp.pair cimport pair
from osmnx_mdp.algorithms.dense_hash_map cimport dense_hash_map
from osmnx_mdp.algorithms.mdp cimport MDP
cimport osmnx_mdp.algorithms.algorithm
from osmnx_mdp.algorithms.brtdp cimport CPP_BRTDP


cdef extern from "cpp_brtdp_replan.hpp":
    struct pair_hash:
        long operator(pair[long, long])

    cppclass CPP_BRTDP_REPLAN(CPP_BRTDP):
        CPP_BRTDP_REPLAN()

        # dense_hash_map[long, double] vl

        # int init(
        #     vector[long] *S,
        #     dense_hash_map[long, vector[pair[long, long]]] *A,
        #     dense_hash_map[pair[long, long], float, pair_hash] *C,
        #     dense_hash_map[long, dense_hash_map[pair[long, long], vector[pair[long, double]], pair_hash]] *P,
        #     dense_hash_map[long, pair[float, float]] *data
        # )
        # int setup(long start, long goal)
        # int run_sample_trial()
        # int run_trials()
        vector[long] get_path(
                dense_hash_map[long, long] diverge_policy,
                float beta,
                bint always_replan)


cdef class BRTDP_REPLAN(osmnx_mdp.algorithms.algorithm.Algorithm):
    cdef dense_hash_map[long, pair[float, float]] data
    cdef dense_hash_map[long, double] vl

    cdef MDP mdp
    cdef CPP_BRTDP_REPLAN cpp

    cdef setup(self, long start, long goal)
    cdef run_trials(self)
    cdef get_path(self, diverge_policy)
    cdef solve(self)
    cdef drive(self, policy, diverge_policy)
