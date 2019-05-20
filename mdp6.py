import time
import sys
import copy
import pickle
from itertools import combinations
from collections import defaultdict
from collections import namedtuple

import osmnx as ox
import numpy as np

from algorithm import Algorithm
from lib import get_angle
from lib import get_edge_cost


# TODO: Put somewhere else or make it a class
Intersection = namedtuple(
    'Intersection',
    ['left_node', 'right_node', 'straight_on_node', 'origin_edge']
)


class MDP(Algorithm):
    def __init__(self, G):
        self.G = ox.project_graph(G)
        self.S = set({})  # [node1, node2, ..]
        self.A = defaultdict(list)  # {node1: [edge1, edge2, ..], ..}
        self.P = defaultdict(dict)  # {node1: {(node1, node_to): [(node_to, 1.0)], ..}, ..}
        self.C = {}  # {node_from: {node_to1: 2, node_to2: 3, ..}, ..}

    def _add_costly_jump_to_goal(self, node):
        # TODO: This seems to be not needed anymore.
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
        # TODO: Consider doing the proper fSSPUDE way.
        # Don't add costly jump to nodes adjacent to the goal.
        if self.goal not in self.G.successors(node):
            self.A[node].append((node, self.goal))
            self.P[node][(node, self.goal)] = [(self.goal, 1.0)]
            self.C[(node, self.goal)] = 100000.

    def _setup(self):
        for node in self.G.nodes():
            self.S.add(node)
            self.A[node] = [edge for edge in self.G.edges(nbunch=node)]

            edges_out = self.G.edges.data(nbunch=node)
            for edge in edges_out:
                action = (edge[0], edge[1])
                self.P[node][action] = self.P[node].get(action, [])
                # For now we end up in correct state 100% of the time.
                self.P[node][action].append((edge[1], 1.0))

        for edge in self.G.edges():
            self.C[edge] = get_edge_cost(self.G, *edge)

    def setup(self, start, goal):
        self.start = start
        self.goal = goal

        self._setup()
        self.make_goal_self_absorbing()
        self.angle_nodes = self.make_low_angle_intersections_uncertain()

        intersections = self.make_close_intersections_uncertain()
        close_nodes = [x.origin_edge[1] for x in intersections]
        self.close_nodes = close_nodes

        total_uncertain_nodes = len(self.angle_nodes) + len(self.close_nodes)
        uncertainty_percent = total_uncertain_nodes / len(self.G.nodes()) * 100
        print('%f%% of nodes are uncertain.' % uncertainty_percent)

    def make_goal_self_absorbing(self):
        """Add a zero-cost loop at the goal to absorb cost.
        """
        self.A[self.goal].append((self.goal, self.goal))
        self.C[(self.goal, self.goal)] = 0
        self.P[self.goal][(self.goal, self.goal)] = [(self.goal, 1.0)]

    def _get_coordinates(self, node):
        return self.G.nodes[node]['x'], self.G.nodes[node]['y']

    def _make_edge_uncertain(self, temp_P, edge, other_node):
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

    def _get_normal_intersection(self, edge):
        origin_node = edge[1]

        straight_on_node = None
        left_node = None
        right_node = None

        for node in self.G.successors(origin_node):
            p1 = self._get_coordinates(edge[0])
            p2 = self._get_coordinates(node)
            origin = self._get_coordinates(origin_node)

            angle = get_angle(p1, p2, origin)
            if angle < 0:
                angle += 360

            if abs(angle - 90) < 10:
                left_node = node

            if abs(angle - 270) < 10:
                right_node = node

            if abs(angle - 180) < 10:
                straight_on_node = node

        if straight_on_node and (right_node or left_node):
            return Intersection(
                left_node,
                right_node,
                straight_on_node,
                edge
            )

    def _get_normal_intersections(self):
        """Scan graph for intersections satisfying the following condition:

        A node with >= 2 outgoing edges is needed with the following
        angles:
            * 180
            * 90 or 270

        Returns a list of nodes that satisfy the condition.
        """
        intersections = []

        for edge in self.G.edges.data():
            intersection = self._get_normal_intersection(edge)
            if intersection:
                intersections.append(intersection)

        return intersections

    def make_close_intersections_uncertain(self, max_length=100):
        """Scan graph for intersections that follow very closely.

        Use cases:
            - When you're supposed to go to the right or left, but go straight
              on because you've missed the intersection as it's very close.
            - When you're supposed to go straight on so that you can turn right
              or left on the next intersection, but you do so on the current
              one, which is too early.
        """
        close_intersections = []

        for intersection in self._get_normal_intersections():
            next_edge = (intersection.origin_edge[1], intersection.straight_on_node)
            next_intersection = self._get_normal_intersection(next_edge)

            if not next_intersection:
                continue

            origin_edge_length = intersection.origin_edge[2]['length']
            next_edge_length = self.G[next_edge[0]][next_edge[1]][0]['length']

            # TODO: Consider ox.clean_intersections, then that <20 check isn't
            # needed anymore.
            if next_edge_length > max_length or origin_edge_length < 20:
                continue

            origin_node = intersection.origin_edge[1]

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
                # TODO NIBBA bad code
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

    def make_low_angle_intersections_uncertain(self, max_angle=30):
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
        # TODO: Code cleanup, esp. _get_coordinates
        # TODO: Docstring
        angle_nodes = []

        for edge in self.G.edges.data():
            p3 = self._get_coordinates(edge[0])
            origin_node = edge[1]
            origin = self._get_coordinates(origin_node)

            edges_out = self.G.edges.data(nbunch=origin_node)
            temp_P = {}

            num_critical_nodes = 0

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

    def _get_Q_value(self, prev_V, gamma, state, action):
        node_to = action[1]

        immediate_cost = 0
        future_cost = 0
        for outcome in self.P[state][action]:
            curr_node_to = outcome[0]
            chance = outcome[1]

            immediate_cost += chance * self.C[(state, node_to)]
            future_cost += chance * gamma * prev_V[curr_node_to]

        return immediate_cost + future_cost

    def solve_value_iteration(
            self,
            gamma=1.,
            max_iter=5000,
            eps=1e-30,
            verbose=True):
        # TODO Add docstring, this is value iteration
        V = {}
        Q = {}
        for s in self.S:
            Q[s] = {a: {} for a in self.A[s]}
            V[s] = 0.

        start = time.time()

        for i in range(max_iter):
            # TODO: This deepcopy is most likely very costly..
            prev_V = copy.deepcopy(V)

            for s in self.S:
                for a in self.A[s]:
                    Q[s][a] = self._get_Q_value(prev_V, gamma, s, a)

                if Q[s]:
                    a = min(Q[s], key=Q[s].get)
                    V[s] = Q[s][a]

            c = sum([abs(x - y) for x, y in zip(prev_V.values(), V.values())])

            # below just debug output
            if verbose:
                if i % 100 == 0:
                    curr_time = time.time() - start
                    sys.stdout.write('\r(%.2f) [%d] %.14f' % (curr_time, i, c))
                    sys.stdout.flush()

            if c <= eps:
                if verbose:
                    print('CONVERGED:', i, c)
                break

        return V, Q

    def get_policy(self, V):
        policy = {}
        for s in self.S:
            actions = []
            for a in self.A[s]:
                v = sum(x[1] * (self.C[(s, a[1])] + V[x[0]]) for x in self.P[s][a])
                actions.append(v)
            if actions:
                policy[s] = np.argmin(actions)

        return policy

    def drive(self, policy, diverge_policy={}):
        # Make sure we don't loop indefinitely due to diverge policy
        visited = set()

        nodes = [self.start]
        curr_node = self.start
        while curr_node != self.goal:
            if curr_node not in policy:
                break

            diverged_node = diverge_policy.get(curr_node, None)

            if diverged_node is None or diverged_node in visited:
                curr_node = self.A[curr_node][policy[curr_node]][1]
            else:
                curr_node = diverged_node
                visited.add(diverged_node)

            nodes.append(curr_node)

        return nodes

    def solve(self):
        V, Q = self.solve_value_iteration()
        return self.get_policy(V)


if __name__ == '__main__':
    with open('data/maxvorstadt.pickle', 'rb') as f:
        G = pickle.load(f)

    mdp = MDP(G)
    mdp.setup(246878841, 372796487)
    V, Q = mdp.solve_value_iteration()

    with open('model5.pickle', 'wb+') as f:
        pickle.dump([mdp, V, Q], f)
