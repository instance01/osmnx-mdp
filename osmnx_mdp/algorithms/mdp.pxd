# cython: language_level=3
cimport osmnx_mdp.algorithms.algorithm

from libcpp.pair cimport pair
from libcpp.vector cimport vector

from osmnx_mdp.algorithms.dense_hash_map cimport dense_hash_map

cdef extern from "cpp_mdp.hpp":
    struct pair_hash:
        long operator(pair[long, long])
    struct CPP_Intersection:
        long left_node
        long right_node
        long straight_on_node
        pair[long, long] origin_edge

    cppclass CPP_MDP:
        CPP_MDP();

        vector[long] *S
        dense_hash_map[long, vector[pair[long, long]]] *A
        dense_hash_map[pair[long, long], float, pair_hash] *C
        dense_hash_map[
            long,
            dense_hash_map[
                pair[long, long],
                vector[pair[long, double]],
                pair_hash
            ]
        ] *P

        dense_hash_map[long, float] V
        dense_hash_map[long, pair[long, long]] policy

        long start
        long goal

        vector[long] angle_nodes
        vector[CPP_Intersection] close_intersections
        dense_hash_map[pair[long, long], double, pair_hash] *edge_data
        dense_hash_map[long, pair[double, double]] *node_data
        dense_hash_map[long, vector[long]] *successors

        int init(
            vector[long] *S,
            dense_hash_map[long, vector[pair[long, long]]] *A,
            dense_hash_map[pair[long, long], float, pair_hash] *C,
            dense_hash_map[
                long,
                dense_hash_map[
                    pair[long, long],
                    vector[pair[long, double]],
                    pair_hash
                ]
            ] *P,
            dense_hash_map[pair[long, long], double, pair_hash] *edge_data,
            dense_hash_map[long, pair[double, double]] *node_data,
            dense_hash_map[long, vector[long]] *successors)
        int setup(long start, long goal)
        int make_goal_self_absorbing()
        int make_edge_uncertain(
            dense_hash_map[pair[long, long], vector[pair[long, double]], pair_hash] &temp_P,
            const pair[long, long] &edge,
            const long &other_node)
        int get_normal_intersections(
                dense_hash_map[pair[long, long], CPP_Intersection, pair_hash] &out)
        int make_close_intersections_uncertain(float max_length=100)
        int make_low_angle_intersections_uncertain(float max_angle=30)
        int solve(int max_iter)
        int get_policy()
        vector[long] drive(dense_hash_map[long, long] &diverge_policy)

cdef class MDP(osmnx_mdp.algorithms.algorithm.Algorithm):
    cdef vector[long] S_
    cdef dense_hash_map[long, vector[pair[long, long]]] A_
    cdef dense_hash_map[pair[long, long], float, pair_hash] C_
    cdef dense_hash_map[
        long,
        dense_hash_map[
            pair[long, long],
            vector[pair[long, double]],
            pair_hash
        ]
    ] P_

    cdef dense_hash_map[pair[long, long], double, pair_hash] edge_data
    cdef dense_hash_map[long, pair[double, double]] node_data
    cdef dense_hash_map[long, vector[long]] successors

    cdef CPP_MDP cpp

    cdef _setup_cpp(self)

    cdef G
    cdef S
    cdef A
    cdef P
    cdef C

    cdef long start
    cdef long goal

    cdef close_nodes
    cdef angle_nodes

    cdef setup(self, long start, long goal)
    cdef solve_value_iteration(self, float gamma=*, int max_iter=*, float eps=*, bint verbose=*)
    cdef get_policy(self, V)
    cdef drive(self, policy, diverge_policy)
    cdef solve(self)
