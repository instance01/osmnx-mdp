import pickle

import matplotlib.pyplot as plt
import osmnx as ox
import networkx as nx

from osmnx_mdp.algorithms.algorithm import Algorithm
from osmnx_mdp.lib import aerial_dist
from osmnx_mdp.lib import get_node_properties
from osmnx_mdp.lib import get_edge_cost


# TODO: unittests


class DStar_Lite(Algorithm):
    def __init__(self, G):
        self.G = ox.project_graph(G)

        # TODO: Explain
        self.k = 0

        # Estimate of distance from current node to start node
        # rhs is an estimate using g value of predecessors + distance from
        # current node to the predecessor.
        # g is the previously calculated g-value (similar to A*).
        # Thus rhs will mostly be a bit ahead of g.
        self.rhs = {}
        self.g = {}

        self.U = {}  # TODO: queue.PriorityQueue()
        self.backup = {}  # TODO: Remove

    def setup(self, start, goal):
        self.start = start
        self.goal = goal

        # TODO Per paper: It's not necessary to init all states to inf here
        # We can also do it when we encounter a new state in
        # compute_shortest_path
        for node in self.G.nodes():
            self.rhs[node] = float('inf')
            self.g[node] = float('inf')

        self.rhs[self.goal] = 0
        self.U[self.goal] = self.calculate_key(self.goal)
        self.backup[self.goal] = self.calculate_key(self.goal)

    def heuristic_func(self, node):
        start_node = self.G.nodes[self.start]
        node = self.G.nodes[node]
        # Divide by 50 since we need an admissible heuristic that doesn't
        # over-estimate, and 50 is optimistic (since in most of the city
        # we have 30 and 50 maxspeed).
        # TODO: However on highways this is not optimistic anymore.
        return aerial_dist(node, start_node) / 50.  # Hours

    def calculate_key(self, node):
        key = min(self.g[node], self.rhs[node])
        return [key + self.heuristic_func(node) + self.k, key]

    def update_vertex(self, u):
        if u != self.goal:
            nodes = list(self.G.successors(u))

            self.rhs[u] = min(
                    self.g[node] + get_edge_cost(self.G, u, node)
                    for node in nodes)
        if u in self.U:
            del self.U[u]

        if self.g[u] != self.rhs[u]:
            self.U[u] = self.calculate_key(u)
            self.backup[u] = self.calculate_key(u)

    def compute_shortest_path(self):
        while len(self.U) > 0 and \
                (min(self.U.values()) < self.calculate_key(self.start) or
                    self.rhs[self.start] != self.g[self.start]):
            u = min(self.U, key=self.U.get)
            k_old = self.U[u]
            del self.U[u]

            key = self.calculate_key(u)
            if k_old < key:
                self.U[u] = key
                self.backup[u] = key
            elif self.g[u] > self.rhs[u]:
                self.g[u] = self.rhs[u]
            else:
                self.g[u] = float('inf')
                self.update_vertex(u)

            for node in self.G.predecessors(u):
                self.update_vertex(node)

    def solve(self):
        self.compute_shortest_path()

    def drive(self, policy, diverge_policy):
        last_start = self.start

        visited = set({})
        visited.add(self.start)

        while self.start != self.goal:
            if self.g[self.start] == float('inf'):
                raise Exception('No path found.')

            diverged_node = diverge_policy.get(self.start, None)
            if diverged_node is None or diverged_node in visited:
                nodes = list(self.G.successors(self.start))
                self.start = min(
                    nodes,
                    key=lambda node: get_edge_cost(self.G, self.start, node) + self.g[node])
                visited.add(self.start)
            else:
                visited.add(diverged_node)
                self.start = diverged_node

                self.k += aerial_dist(
                        self.G.nodes[last_start],
                        self.G.nodes[self.start])
                last_start = self.start
                self.compute_shortest_path()

        return visited


if __name__ == '__main__':
    with open('data/maxvorstadt.pickle', 'rb') as f:
        G = pickle.load(f)

    dstar = DStar_Lite(G)
    dstar.setup(246878841, 372796487)
    dstar.compute_shortest_path()
    col_nodes = list(dstar.drive({}, {}))
    print(col_nodes)

    x = {}
    for k, v in dstar.backup.items():
        x[k] = v[0]

    nc, ns = get_node_properties(dstar.G, col_nodes, x)
    for node in dstar.G.nodes(data=True):
        dstar.G.node[node[0]]['pos'] = (node[1]['x'], node[1]['y'])
    nx.draw_networkx(
            dstar.G,
            nx.get_node_attributes(dstar.G, 'pos'),
            node_size=ns,
            node_color=nc,
            node_zorder=2,
            with_labels=False)

    plt.show()
