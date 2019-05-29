# cython: language_level=3
# cython: profile=True
from libcpp.pair cimport pair
from libcpp.vector cimport vector

from osmnx_mdp.algorithms.dense_hash_map cimport dense_hash_map


cdef extern from "cpp_mdp.hpp":
    struct pair_hash:
        long operator(pair[long, long])
    void solve(
        dense_hash_map[long, float] &V,
        vector[long] &S,
        dense_hash_map[long, vector[pair[long, long]]] &A,
        dense_hash_map[pair[long, long], float, pair_hash] &C,
        dense_hash_map[long, dense_hash_map[pair[long, long], vector[pair[long, float]], pair_hash]] &P,
        int max_iter
    )
    struct CPP_Intersection:
        long left_node
        long right_node
        long straight_on_node
        pair[long, long] origin_edge
    void CPP_get_normal_intersections(
        dense_hash_map[pair[long, long], CPP_Intersection, pair_hash] &out,
        dense_hash_map[long, vector[long]] &successors,
        dense_hash_map[long, pair[float, float]] &data
    )


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

cimport cython


Intersection = namedtuple(
    'Intersection',
    ['left_node', 'right_node', 'straight_on_node', 'origin_edge']
)


cdef class MDP(osmnx_mdp.algorithms.algorithm.Algorithm):
    def __reduce__(self):
        return (self.__class__, (self.G,), {
            "G": self.G,
            "S": self.S,
            "A": self.A,
            "P": self.P,
            "C": self.C,
            "start": self.start,
            "goal": self.goal,
            "close_nodes": self.close_nodes,
            "angle_nodes": self.angle_nodes})

    def __cinit__(self, G):
        self.__init__(G)

    def __init__(self, G):
        self.G = G
        self.S = set({})  # [node1, node2, ..]
        self.A = defaultdict(list)  # {node1: [edge1, edge2, ..], ..}
        self.P = defaultdict(dict)  # {node1: {(node1, node_to): [(node_to, 1.0)], ..}, ..}
        self.C = {}  # {node_from: {node_to1: 2, node_to2: 3, ..}, ..}

    cdef _add_costly_jump_to_goal(self, node):
        # TODO: This seems to be not needed anymore, remove.
        """This is to avoid dead ends.
        We do this by adding a very costly jump to the goal to each node.
        If a node ends up being a dead end, i.e. doesn't have any outgoing
        edges except this one costly jump, its cost becomes as huge.
        Thus this node becomes undesirable and will be avoided in the future.
        See proof sketch of theorem 3 in the paper:
            Kobolov, Stochastic Shortest Path MDPs with Dead Ends.
        Another way would be to simply set V of a dead end node (which is not
        the goal node) to a very high cost, as described in the definition of
        fSSPUDE (same paper as above).
        """
        # Don't add costly jump to nodes adjacent to the goal.
        if self.goal not in self.G.successors(node):
            self.A[node].append((node, self.goal))
            self.P[node][(node, self.goal)] = [(self.goal, 1.0)]
            self.C[(node, self.goal)] = 100000.

    cdef _setup(self):
        for node in self.G.nodes():
            successors = list(self.G.successors(node))

            self.S.add(node)
            self.A[node] = [(node, succ) for succ in successors]

            for succ in successors:
                action = (node, succ)
                self.P[node][action] = self.P[node].get(action, [])
                # For now we end up in correct state 100% of the time.
                self.P[node][action].append((succ, 1.0))
                self.C[action] = get_edge_cost(self.G, node, succ)

    cdef setup(self, long start, long goal):
        self.start = start
        self.goal = goal

        self._setup()
        self.make_goal_self_absorbing()
        self.angle_nodes = self.make_low_angle_intersections_uncertain()

        intersections = self.make_close_intersections_uncertain()
        cdef CPP_Intersection x
        close_nodes = [x.origin_edge.second for x in intersections]
        self.close_nodes = close_nodes

        total_uncertain_nodes = len(self.angle_nodes) + len(self.close_nodes)
        uncertainty_percent = total_uncertain_nodes / self.G.number_of_nodes() * 100
        print('%f%% of nodes are uncertain.' % uncertainty_percent)

    cdef make_goal_self_absorbing(self):
        """Add a zero-cost loop at the goal to absorb cost.
        """
        self.A[self.goal].append((self.goal, self.goal))
        self.C[(self.goal, self.goal)] = 0
        self.P[self.goal][(self.goal, self.goal)] = [(self.goal, 1.0)]

    cdef _get_coordinates(self, node):
        return self.G.nodes[node]['x'], self.G.nodes[node]['y']

    cdef _make_edge_uncertain(self, temp_P, edge, other_node):
        """Make taking the action of following given edge probabilistic,
        i.e. end up in the expected edge only 90% of the time and end
        up in other_node 10% of the time.
        If the edge is already probabilistic, decrease its chance (e.g.
        from 90% to 80% and so on).
        Modifies temp_P inplace.
        """
        node_to = edge[1]

        if edge not in temp_P:
            temp_P[edge] = [(node_to, .9), (other_node, .1)]
        else:
            temp_P[edge].append((other_node, .1))
            temp_P[edge][0] = (node_to, temp_P[edge][0][1] - .1)

    #cdef _get_normal_intersection(self, edge):
    #    origin_node = edge[1]

    #    straight_on_node = None
    #    left_node = None
    #    right_node = None

    #    for node in self.G.successors(origin_node):
    #        p1 = self._get_coordinates(edge[0])
    #        p2 = self._get_coordinates(node)
    #        origin = self._get_coordinates(origin_node)

    #        angle = get_angle(p1, p2, origin)
    #        if angle < 0:
    #            angle += 360

    #        if abs(angle - 90) < 10:
    #            left_node = node

    #        if abs(angle - 270) < 10:
    #            right_node = node

    #        if abs(angle - 180) < 10:
    #            straight_on_node = node

    #    if straight_on_node and (right_node or left_node):
    #        return Intersection(
    #            left_node,
    #            right_node,
    #            straight_on_node,
    #            edge
    #        )

    cdef _get_normal_intersections(self):
        """Scan graph for intersections satisfying the following condition:

        A node with >= 2 outgoing edges is needed with the following
        angles:
            * 180
            * 90 or 270

        Returns a list of nodes that satisfy the condition.
        """

        # TODO Move this into setup and put into self.
        # So we can refer to it from everywhere
        cdef dense_hash_map[long, vector[long]] successors
        cdef dense_hash_map[long, pair[float, float]] data

        successors.set_empty_key(0)
        data.set_empty_key(0)

        for node in self.G.nodes.data():
            successors[node[0]] = list(self.G.successors(node[0]))
            data[node[0]] = (node[1]['x'], node[1]['y'])

        cdef dense_hash_map[pair[long, long], CPP_Intersection, pair_hash] out
        CPP_get_normal_intersections(
            out,
            successors,
            data
        )

        ret = {}
        cdef CPP_Intersection v
        for k, v in out:
            ret[k] = v

        return ret

    cdef make_close_intersections_uncertain(self, max_length=100):
        """Scan graph for intersections that follow very closely.

        Use cases:
            - When you're supposed to go to the right or left, but go straight
              on because you've missed the intersection as it's very close.
            - When you're supposed to go straight on so that you can turn right
              or left on the next intersection, but you do so on the current
              one, which is too early.
        """
        close_intersections = []

        intersections = self._get_normal_intersections()

        cdef CPP_Intersection intersection
        cdef CPP_Intersection next_intersection
        for _, intersection in intersections.items():
            next_edge = (intersection.origin_edge.second, intersection.straight_on_node)
            # TODO: We're still using slow python dict here..
            if next_edge in intersections:
                next_intersection = intersections[next_edge]
            else:
                continue
            origin_edge = (intersection.origin_edge.first, intersection.origin_edge.second)

            #origin_edge_length = intersection.origin_edge[2]['length']
            origin_edge_length = self.G[origin_edge[0]][origin_edge[1]][0]['length']
            next_edge_length = self.G[next_edge[0]][next_edge[1]][0]['length']

            # TODO: Consider ox.clean_intersections, then that <20 check isn't
            # needed anymore.
            if next_edge_length > max_length or origin_edge_length < 20:
                continue

            origin_node = origin_edge[1]

            # TODO Cleanup below

            if (intersection.left_node and next_intersection.left_node):
                close_intersections.append(intersection)
                self._make_edge_uncertain(
                        self.P[origin_node],
                        next_edge,
                        intersection.left_node)
                self._make_edge_uncertain(
                        self.P[origin_node],
                        (origin_node, intersection.left_node),
                        intersection.straight_on_node)

            if (intersection.right_node and next_intersection.right_node):
                # TODO: Improve code
                if intersection not in close_intersections:
                    close_intersections.append(intersection)
                self._make_edge_uncertain(
                        self.P[origin_node],
                        next_edge,
                        intersection.right_node)
                self._make_edge_uncertain(
                        self.P[origin_node],
                        (origin_node, intersection.right_node),
                        intersection.straight_on_node)

        return close_intersections

    cdef make_low_angle_intersections_uncertain(self, max_angle=30):
        """
         (2)   (3)
          *     *
           \   /
            \ /
             * (1)

        If angle between edges (1, 2) and (1, 3) is small enough,
        make those edges uncertain, i.e. add a 10% chance end up
        in the other node and not the expected one.
        """
        angle_nodes = []

        #cdef dense_hash_map[long, vector[long]] successors
        #cdef dense_hash_map[long, pair[float, float]] data

        #successors.set_empty_key(0)
        #data.set_empty_key(0)

        #for node in self.G.nodes.data():
        #    successors[node[0]] = list(self.G.successors(node[0]))
        #    data[node[0]] = (node[1]['x'], node[1]['y'])

        for edge in self.G.edges.data():
            p3 = self._get_coordinates(edge[0])
            origin_node = edge[1]
            origin = self._get_coordinates(origin_node)

            edges_out = self.G.edges.data(nbunch=origin_node)
            temp_P = {}

            num_critical_nodes = 0


            # TODO: How to model combinations in C++ ?!

            from itertools import tee
            def pairwise(iterable):
                "s -> (s0,s1), (s1,s2), (s2, s3), ..."
                a, b = tee(iterable)
                next(b, None)
                return zip(a, b)


            #for (e1, e2) in combinations(edges_out, 2):
            for (e1, e2) in combinations(edges_out, 2):
                # We can't reuse the edge that goes back as one of the
                # out_edges.
                # 1   3
                #  \  |
                #   \ |
                #     2
                # We come from 1, and without the checks below we reuse
                # edge (1, 2) and erroneously find a critical angle
                # between (1, 2) and (2, 3).
                if e1[:2] == edge[1::-1] or e2[:2] == edge[1::-1]:
                    continue

                p1 = self._get_coordinates(e1[1])
                p2 = self._get_coordinates(e2[1])

                # The following line solves the following scenario:
                # 1   3   4
                #  \  |  /
                #   \ | /
                #     2
                # Let's say we come from node 3 and we're currently at node 2.
                # We used to simply get the angle between the two edges (here
                # (1, 2) and (4, 2)), which in this case is <30 degrees.
                # But this is a T-shaped intersection, just sharper. This is
                # not a critical intersection.
                # So currently we get the angle between edge (4, 2) and (3, 2),
                # which is e.g. 20 degrees, and the angle between (1, 2) and
                # (3, 2), which is then 340 degrees.
                # It follows that the difference of both is 320 > 30 degrees.
                # Thus it is not a critical intersection.
                angle = get_angle(p1, p3, origin) - get_angle(p2, p3, origin)
                if abs(angle) <= max_angle:
                    edge1 = e1[:2]
                    edge2 = e2[:2]

                    self._make_edge_uncertain(temp_P, edge1, edge2[1])
                    self._make_edge_uncertain(temp_P, edge2, edge1[1])

                    num_critical_nodes += 1

            if num_critical_nodes > 0:
                angle_nodes.append(origin_node)

            self.P[origin_node].update(temp_P)

        return angle_nodes

    cdef _get_Q_value(self, prev_V, gamma, state, action):
        cdef float immediate_cost = self.C[action]

        # sum() is slower than a for loop, because the array is usually only
        # a few elements (we usually have only 1 or 2 outcomes).
        # So the overhead of creating a new array is too big.
        # future_cost = sum(
        #     chance * gamma * prev_V[next_node]
        #     for next_node, chance in self.P[state][action]
        # )

        cdef float future_cost = 0
        for next_node, chance in self.P[state][action]:
            future_cost += chance * gamma * prev_V[next_node]

        return immediate_cost + future_cost

    cdef solve_value_iteration(
            self,
            float gamma=1.,
            int max_iter=5000,
            float eps=1e-30,
            bint verbose=True):
        """Solve the MDP with value iteration.
        """
        # Direct assignment is not possible since Cython doesnt know how to
        # convert a dense_hash_map to a Python object.
        # But iterators work.

        cdef dense_hash_map[long, float] V
        V.set_empty_key(0)

        cdef vector[long] S = self.S

        cdef dense_hash_map[long, vector[pair[long, long]]] A
        A.set_empty_key(0)
        for k, v in self.A.items():
            A[k] = v

        cdef dense_hash_map[pair[long, long], float, pair_hash] C
        C.set_empty_key((0, 0))
        for k, v in self.C.items():
            C[k] = v

        cdef dense_hash_map[long, dense_hash_map[pair[long, long], vector[pair[long, float]], pair_hash]] P
        P.set_empty_key(0)
        cdef dense_hash_map[pair[long, long], vector[pair[long, float]], pair_hash] curr
        curr.set_empty_key((0, 0))
        for k, v in self.P.items():
            for k_, v_ in v.items():
                curr[k_] = v_
            P[k] = curr
            curr.clear()


        solve(V, S, A, C, P, max_iter)

        #return V, {}
        ret = {}
        for k, v in V:
            ret[k] = v
        return ret, {}

    cdef get_policy(self, V):
        policy = {}
        for s in self.S:
            actions = []
            for a in self.A[s]:
                v = sum([x[1] * (self.C[(s, a[1])] + V[x[0]]) for x in self.P[s][a]])
                actions.append(v)
            if actions:
                policy[s] = np.argmin(actions)

        return policy

    cdef drive(self, policy, diverge_policy):
        # Make sure we don't loop indefinitely due to diverge policy
        visited = set()

        nodes = [self.start]
        nodes_lookup = set(nodes)
        curr_node = self.start
        while curr_node != self.goal:
            if curr_node not in policy:
                break

            diverged_node = diverge_policy.get(curr_node, None)

            going_backward = diverged_node in visited or diverged_node in nodes_lookup
            if diverged_node is None or going_backward:
                curr_node = self.A[curr_node][policy[curr_node]][1]
            else:
                curr_node = diverged_node
                visited.add(diverged_node)

            nodes.append(curr_node)

        return nodes

    cdef solve(self):
        V, _ = self.solve_value_iteration()
        return self.get_policy(V)
