# cython: language_level=3
cdef class Algorithm:
    cdef setup(self, long start, long goal)
    cdef solve(self)
    cdef drive(self, policy, diverge_policy)