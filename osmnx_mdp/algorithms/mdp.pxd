# cython: language_level=3
cimport osmnx_mdp.algorithms.algorithm

cdef class MDP(osmnx_mdp.algorithms.algorithm.Algorithm):
    cdef G
    cdef S
    cdef A
    cdef P
    cdef C

    cdef long start
    cdef long goal

    cdef close_nodes
    cdef angle_nodes

    cdef _add_costly_jump_to_goal(self, node)
    cdef _setup(self)
    cdef setup(self, long start, long goal)
    cdef make_goal_self_absorbing(self)
    cdef _get_coordinates(self, node)
    cdef _make_edge_uncertain(self, temp_P, edge, other_node)
    #cdef _get_normal_intersection(self, edge)
    cdef _get_normal_intersections(self)
    cdef make_close_intersections_uncertain(self, max_length=*)
    cdef make_low_angle_intersections_uncertain(self, max_angle=*)
    cdef _get_Q_value(self, prev_V, gamma, state, action)
    cdef solve_value_iteration(self, float gamma=*, int max_iter=*, float eps=*, bint verbose=*)
    cdef get_policy(self, V)
    cdef drive(self, policy, diverge_policy)
    cdef solve(self)
