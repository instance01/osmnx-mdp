import time
import sys
import copy
import pickle
from itertools import combinations
from collections import defaultdict

import osmnx as ox
import numpy as np

from lib import get_angle
from lib import get_edge_cost


class MDP(object):
    def __init__(self, G):
        self.G = ox.project_graph(G)
        self.S = set({})  # [node1, node2, ..]
        self.A = defaultdict(list)  # {node1: [edge1, edge2, ..], ..}
        self.P = defaultdict(dict)  # {node1: {(node1, node_to): [(node_to, 1.0)], ..}, ..}
        self.C = {}  # {node_from: {node_to1: 2, node_to2: 3, ..}, ..}
        self.start = 246878841
        self.goal = 372796487

    def add_costly_jump_to_goal(self, node):
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
        self.A[node].append((node, self.goal))
        self.P[node][(node, self.goal)] = [(self.goal, 1.0)]
        self.C[(node, self.goal)] = 100000.

    def setup(self):
        for node in self.G.nodes():
            self.S.add(node)
            self.A[node] = [edge for edge in self.G.edges(nbunch=node)]

            edges_out = self.G.edges.data(nbunch=node)
            for edge in edges_out:
                action = (edge[0], edge[1])
                self.P[node][action] = self.P[node].get(action, [])
                # For now we end up in correct state 100% of the time.
                self.P[node][action].append((edge[1], 1.0))

            # This is to avoid dead ends.
            self.add_costly_jump_to_goal(node)
            # TODO: BTW we overwrite costs of nodes adjacent to goal in lines
            # below, so order is important here. That's bad code, fix.
            # TODO Add comment regarding this

        for edge in self.G.edges():
            self.C[edge] = get_edge_cost(self.G, *edge)

    def make_goal_self_absorbing(self):
        """Add a zero-cost loop at the goal to absorb cost.
        """
        self.A[self.goal].append((self.goal, self.goal))
        self.C[(self.goal, self.goal)] = 0
        self.P[self.goal][(self.goal, self.goal)] = [(self.goal, 1.0)]

    def remove_zero_cost_loops(self):
        """Apparently osmnx gives us looping edges (that go back to itself) on
        the border of the map. The problem is that they're zero-cost loops.
        Let's just remove them.
        """
        loops = [edge for edge in self.G.edges() if edge[0] == edge[1]]
        for loop in loops:
            self.G.remove_edge(*loop)

    def remove_dead_ends(self):
        """Since the map (in current tests Maxvorstadt) is 'cut out' of Munich,
        some paths leave the map without coming back. This means if a driver
        were to follow this path, he would not be able to come back, so this
        is a dead end.
        Value iteration does not converge with dead ends.
        Since we know where our dead ends lie, we can simply filter them out
        and don't have to resort to other models such as fSSPUDE etc. TODO
        Paper: Kolobov, Stochastic Shortest Path MDPs with Dead Ends
        """
        while True:
            # TODO: This possibly doesn't have to be a set.
            todel = set()

            candidates = []
            for node in self.G.nodes():
                if node == self.goal:
                    continue

                predecessors = list(self.G.predecessors(node))
                successors = list(self.G.successors(node))

                has_loop = predecessors == successors and len(predecessors) == 1
                if not successors or has_loop:
                    candidates.append(node)

            for candidate in candidates:
                open_ = set(self.G.predecessors(candidate))
                open_.add(candidate)

                while open_:
                    node = open_.pop()

                    if node in todel:
                        continue

                    successors = list(self.G.successors(node))
                    if len(successors) <= 1:
                        open_.update(self.G.predecessors(node))
                        todel.add(node)

            if not todel:
                break

            for node in todel:
                self.G.remove_node(node)

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
            temp_P[edge][0] = (node_to, temp_P[edge][0] - .1)

    def update_uncertain_intersections(self, max_angle=30):
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
                    #  (2)   (3)
                    #   *     *
                    #    \   /
                    #     \ /
                    #      * (1)

                    # If angle between edges (1, 2) and (1, 3) is small enough,
                    # make those edges uncertain, i.e. add a 10% chance end up
                    # in the other node and not the expected one.
                    edge1 = e1[:2]
                    edge2 = e2[:2]

                    self._make_edge_uncertain(temp_P, edge1, edge2[1])
                    self._make_edge_uncertain(temp_P, edge2, edge1[1])

                    num_critical_nodes += 1
            if num_critical_nodes > 0:
                angle_nodes.append(origin_node)
            self.P[origin_node].update(temp_P)

        return angle_nodes

    def get_Q_value(self, prev_V, gamma, state, action):
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
                    Q[s][a] = self.get_Q_value(prev_V, gamma, s, a)

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

    def drive(self, policy, start=None, goal=None):
        if start is None:
            start = self.start
        if goal is None:
            goal = self.goal

        nodes = [start]
        curr_node = start
        while curr_node != goal:
            if curr_node not in policy:
                break
            curr_node = self.A[curr_node][policy[curr_node]][1]
            nodes.append(curr_node)

        return nodes


if __name__ == '__main__':
    with open('maxvorstadt2.pickle', 'rb') as f:
        G = pickle.load(f)

    mdp = MDP(G)
    mdp.remove_zero_cost_loops()
    mdp.remove_dead_ends()
    mdp.setup()
    mdp.make_goal_self_absorbing()  # TODO: This could be moved into setup()
    mdp.angle_nodes = mdp.update_uncertain_intersections()
    V, Q = mdp.solve_value_iteration()

    with open('model5.pickle', 'wb+') as f:
        pickle.dump([mdp, V, Q], f)
