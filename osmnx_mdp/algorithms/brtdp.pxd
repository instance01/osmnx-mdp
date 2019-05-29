# cython: language_level=3
from libcpp.vector cimport vector
from libcpp.pair cimport pair
from osmnx_mdp.algorithms.dense_hash_map cimport dense_hash_map
from osmnx_mdp.algorithms.mdp cimport MDP


cdef extern from "cpp_brtdp.cpp":
    struct pair_hash:
        long operator(pair[long, long])

    cppclass CPP_BRTDP:
        CPP_BRTDP()

        dense_hash_map[long, double] vl

        int init(
            vector[long] *S,
            dense_hash_map[long, vector[pair[long, long]]] *A,
            dense_hash_map[pair[long, long], float, pair_hash] *C,
            dense_hash_map[long, dense_hash_map[pair[long, long], vector[pair[long, double]], pair_hash]] *P,
            dense_hash_map[long, pair[float, float]] *data
        )
        int setup(long start, long goal)
        int run_sample_trial()
        int run_trials()
        vector[long] get_path()


cdef class BRTDP:
    cdef dense_hash_map[long, pair[float, float]] data
    cdef vector[long] S
    cdef dense_hash_map[long, vector[pair[long, long]]] A
    cdef dense_hash_map[pair[long, long], float, pair_hash] C
    cdef dense_hash_map[long, dense_hash_map[pair[long, long], vector[pair[long, double]], pair_hash]] P

    cdef dense_hash_map[long, double] vl

    cdef MDP mdp
    cdef CPP_BRTDP cpp

    cdef setup(self, long start, long goal)
    cdef run_trials(self)
    cdef get_path(self)
