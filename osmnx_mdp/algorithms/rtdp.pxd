# cython: language_level=3
cimport osmnx_mdp.algorithms.algorithm
cimport osmnx_mdp.algorithms.mdp

cdef class RTDP(osmnx_mdp.algorithms.algorithm.Algorithm):
    cdef osmnx_mdp.algorithms.mdp.MDP mdp
    cdef G
    cdef H
    cdef h

    cdef Q

    cdef long start
    cdef long goal

    cdef setup(self, long start, long goal)
    cdef _select_next_state(self, state, action)
    cdef _run_trial(self)
    cdef run_trials(self, n=*)
    cdef solve(self)
    cdef drive(self, policy, diverge_policy)
