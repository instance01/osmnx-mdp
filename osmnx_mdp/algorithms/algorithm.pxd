# cython: language_level=3
from libcpp.unordered_map cimport unordered_map
from libcpp.string cimport string

cdef class Algorithm:
    cdef int iterations
    cdef setup(self, long start, long goal, unordered_map[string, double] cfg)
    cdef solve(self)
    cdef drive(self, policy, diverge_policy)
