# cython: language_level=3
cdef class Algorithm:
    cdef setup(self, long start, long goal):
        pass

    cdef solve(self):
        pass

    cdef drive(self, policy, diverge_policy):
        pass
