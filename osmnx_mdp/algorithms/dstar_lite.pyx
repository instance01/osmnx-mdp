# cython: language_level=3
from libc.math cimport INFINITY
from libcpp.pair cimport pair
from libcpp.set cimport set as cset
from libcpp.vector cimport vector

import pickle

import matplotlib.pyplot as plt
import osmnx as ox
import networkx as nx

from osmnx_mdp.lib cimport aerial_dist
from osmnx_mdp.lib cimport get_node_properties
from osmnx_mdp.lib cimport get_edge_cost

cimport osmnx_mdp.algorithms.algorithm

from osmnx_mdp.algorithms.dense_hash_map cimport dense_hash_map


# TODO: unittests
# This is a wrapper for the C++ module.


cdef class DStar_Lite(osmnx_mdp.algorithms.algorithm.Algorithm):
    def __init__(self, G):
        self.G = G

    cdef setup(self, long start, long goal):
        self.cost.set_empty_key((0, 0))
        self.data.set_empty_key(0)
        self.successors.set_empty_key(0)
        self.predecessors.set_empty_key(0)

        # TODO Unreadable
        for node in self.G.nodes.data():
            node_id = node[0]

            node_successors = list(self.G.successors(node_id))
            node_predecessors = list(self.G.predecessors(node_id))

            self.successors[node_id] = node_successors
            self.predecessors[node_id] = node_predecessors
            self.data[node_id] = (node[1]['lat'], node[1]['lon'])
            for succ in node_successors:
                self.cost[(node_id, succ)] = get_edge_cost(self.G, node_id, succ)

        self.cpp = cpp_DStar_Lite()
        self.cpp.init(&self.cost, &self.data, &self.predecessors, &self.successors)
        self.cpp.setup(start, goal)

    cdef heuristic_func(self, node):
        return self.cpp.heuristic_func(node)

    cdef calculate_key(self, node):
        return self.cpp.calculate_key(node)

    cdef update_vertex(self, u):
        return self.cpp.update_vertex(u)

    cdef compute_shortest_path(self):
        self.cpp.compute_shortest_path()

    cdef solve(self):
        self.compute_shortest_path()

    cdef drive(self, policy, diverge_policy):
        cdef dense_hash_map[long, long] diverge_policy_cpp
        diverge_policy_cpp.set_empty_key(0)
        for k, v in diverge_policy:
            diverge_policy_cpp[k] = v

        cdef cset[long] path
        self.cpp.drive(path, diverge_policy_cpp)
        return path
