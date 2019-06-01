# cython: language_level=3
# cython: profile=True
from libcpp.pair cimport pair
from libcpp.vector cimport vector

from osmnx_mdp.algorithms.dense_hash_map cimport dense_hash_map

import time
import sys
import copy
import pickle
from itertools import combinations
from collections import defaultdict
from collections import namedtuple

import numpy as np

from osmnx_mdp.lib cimport get_angle
from osmnx_mdp.lib cimport get_edge_cost
from osmnx_mdp.lib cimport remove_dead_ends


Intersection = namedtuple(
    'Intersection',
    ['left_node', 'right_node', 'straight_on_node', 'origin_edge']
)


cdef class MDP(osmnx_mdp.algorithms.algorithm.Algorithm):
    def __init__(self, G):
        self.G = G
        self.cpp = CPP_MDP()

    cdef _setup_cpp(self):
        self.successors.set_empty_key(0)
        self.node_data.set_empty_key(0)
        self.edge_data.set_empty_key((0, 0))
        self.A_.set_empty_key(0)
        self.P_.set_empty_key(0)
        self.C_.set_empty_key((0, 0))

        for node in self.G.nodes.data():
            node_id = node[0]
            successors = list(self.G.successors(node_id))

            self.successors[node_id] = successors

            self.node_data[node_id] = (node[1]['x'], node[1]['y'])

            self.S_.push_back(node_id)
            self.A_[node_id] = [(node_id, succ) for succ in successors]

            if self.P_.find(node_id) == self.P_.end():
                self.P_[node_id].set_empty_key((0, 0))

            for succ in successors:
                action = (node_id, succ)
                # For now we end up in correct state 100% of the time.
                self.P_[node_id][action].push_back((succ, 1.0))
                self.C_[action] = get_edge_cost(self.G, node_id, succ)

                self.edge_data[(node_id, succ)] = self.G[node_id][succ][0]['length']

    cdef setup(self, long start, long goal):
        self._setup_cpp()
        self.cpp.init(
                &self.S_,
                &self.A_,
                &self.C_,
                &self.P_,
                &self.edge_data,
                &self.node_data,
                &self.successors)
        self.cpp.setup(start, goal)

    cdef solve_value_iteration(
            self,
            float gamma=1.,
            int max_iter=5000,
            float eps=1e-30,
            bint verbose=True):
        """Solve the MDP with value iteration.
        """
        self.cpp.solve(max_iter)

        # Direct assignment is not possible since Cython doesnt know how to
        # convert a dense_hash_map to a Python object.
        # But iterators work.
        # TODO: Find a way to make this not ugly.
        V = {}
        for k, v in self.cpp.V:
            V[k] = v

        return V, {}

    cdef get_policy(self, V):
        self.cpp.get_policy()

        # Manually unwrap Cython object into Python object.
        policy = {}
        for k, v in self.cpp.policy:
            policy[k] = v
        return policy

    cdef drive(self, policy, diverge_policy):
        # Make sure we don't loop indefinitely due to diverge policy
        cdef dense_hash_map[long, long] cpp_diverge_policy
        cpp_diverge_policy.set_empty_key(0)

        # Manually wrap Python object into Cython object.
        for k, v in diverge_policy:
            cpp_diverge_policy[k] = v

        return self.cpp.drive(cpp_diverge_policy)

    cdef solve(self):
        V, _ = self.solve_value_iteration()
        return self.get_policy(V)
