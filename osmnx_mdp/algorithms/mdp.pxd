# cython: language_level=3
cimport osmnx_mdp.algorithms.algorithm

from libcpp.string cimport string
from libcpp.pair cimport pair
from libcpp.vector cimport vector
from libcpp.unordered_map cimport unordered_map
from libcpp.set cimport set as cset

from osmnx_mdp.algorithms.dense_hash_map cimport dense_hash_map


cdef extern from "cpp_mdp.hpp":
    struct pair_hash:
        long operator(pair[long, long])
    struct CPP_Intersection "Intersection":
        long left_node
        long right_node
        long straight_on_node
        pair[long, long] origin_edge

    cppclass CPP_MDP "MDP":
        CPP_MDP();

        vector[long] *S
        dense_hash_map[long, vector[pair[long, long]]] *A
        dense_hash_map[pair[long, long], double, pair_hash] *C
        dense_hash_map[
            long,
            dense_hash_map[
                pair[long, long],
                vector[pair[long, double]],
                pair_hash
            ]
        ] *P

        dense_hash_map[pair[long, long], double, pair_hash] V
        dense_hash_map[pair[long, long], pair[long, long], pair_hash] policy

        long start
        long goal

        vector[long] angle_nodes
        vector[CPP_Intersection] close_intersections
        dense_hash_map[pair[long, long], double, pair_hash] *edge_data
        dense_hash_map[long, pair[double, double]] *node_data
        dense_hash_map[pair[long, long], pair[double, double], pair_hash] *angle_data
        dense_hash_map[long, vector[long]] *predecessors
        dense_hash_map[long, vector[long]] *successors

        cset[long] uncertain_nodes

        int init(
            vector[long] *S,
            dense_hash_map[long, vector[pair[long, long]]] *A,
            dense_hash_map[pair[long, long], double, pair_hash] *C,
            dense_hash_map[
                long,
                dense_hash_map[
                    pair[long, long],
                    vector[pair[long, double]],
                    pair_hash
                ]
            ] *P,
            dense_hash_map[pair[long, long], double, pair_hash] *edge_data,
            dense_hash_map[pair[long, long], pair[double, double], pair_hash] *angle_data,
            dense_hash_map[long, pair[double, double]] *node_data,
            dense_hash_map[long, vector[long]] *successors,
            dense_hash_map[long, vector[long]] *predecessors)
        int setup(long start, long goal, unordered_map[string, double] cfg)
        int make_goal_self_absorbing()
        int make_edge_uncertain(
            dense_hash_map[pair[long, long], vector[pair[long, double]], pair_hash] &temp_P,
            const pair[long, long] &edge,
            const long &other_node)
        int get_normal_intersections(
                dense_hash_map[pair[long, long], CPP_Intersection, pair_hash] &out)
        int make_close_intersections_uncertain(double max_length=100)
        int make_low_angle_intersections_uncertain(double max_angle=30)
        int solve(int max_iter, double eps)
        int get_policy()
        vector[long] drive(dense_hash_map[long, long] &diverge_policy)

cdef class MDP(osmnx_mdp.algorithms.algorithm.Algorithm):
    cdef vector[long] S
    cdef dense_hash_map[long, vector[pair[long, long]]] A
    cdef dense_hash_map[pair[long, long], double, pair_hash] C
    cdef dense_hash_map[
        long,
        dense_hash_map[
            pair[long, long],
            vector[pair[long, double]],
            pair_hash
        ]
    ] P

    cdef dense_hash_map[pair[long, long], double, pair_hash] edge_data
    cdef dense_hash_map[long, pair[double, double]] node_data
    cdef dense_hash_map[pair[long, long], pair[double, double], pair_hash] angle_data
    cdef dense_hash_map[long, vector[long]] predecessors
    cdef dense_hash_map[long, vector[long]] successors

    cdef CPP_MDP cpp
    cdef G

    cdef long start
    cdef long goal

    cdef close_nodes
    cdef angle_nodes

    cdef set uncertain_nodes

    cdef _setup_cpp(self)
    cdef setup(self, long start, long goal, unordered_map[string, double] cfg)
    cdef solve_value_iteration(self, double gamma=*, int max_iter=*, double eps=*, bint verbose=*)
    cdef get_policy(self, V)
    cdef drive(self, policy, diverge_policy)
    cdef solve(self)
