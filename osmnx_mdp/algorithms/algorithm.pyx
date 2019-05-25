# cython: language_level=3
cdef class Algorithm:
    def __init__(self, G):
        pass

    def __cinit__(self, G):
        pass

    cdef setup(self, long start, long goal):
        pass

    cdef solve(self):
        pass

    cdef drive(self, policy, diverge_policy):
        pass
