import pickle

import matplotlib.pyplot as plt
import osmnx as ox
import networkx as nx

from algorithm import Algorithm
from lib import aerial_dist
from lib import get_node_properties


# TODO: unittests


class LPA_Star(Algorithm):
    def __init__(self, G):
        self.G = G
        # TODO Don't hardcode start/goal lmao
        self.start = 246878841
        self.goal = 372796487

        # Estimate of distance from current node to start node
        # rhs is an estimate using g value of predecessors + distance from
        # current node to the predecessor.
        # g is the previously calculated g-value (similar to A*).
        # Thus rhs will mostly be a bit ahead of g.
        self.rhs = {}
        self.g = {}

        self.U = {}  # TODO: queue.PriorityQueue()
        self.backup = {}  # TODO: REMOVE

    def heuristic_func(self, node):
        goal_node = self.G_projected.nodes[self.goal]
        node = self.G_projected.nodes[node]
        return aerial_dist(node, goal_node) / 50.  # In hours, optimistic

    def calculate_key(self, node):
        key = min(self.g[node], self.rhs[node])
        # f value and min(g, rhs)
        return [key + self.heuristic_func(node), key]

    def init(self):
        self.G_projected = ox.project_graph(self.G)

        # TODO Per paper: It's not necessary to init all states to inf here
        # We can also do it when we encounter a new state in
        # compute_shortest_path
        for node in self.G.nodes():
            self.rhs[node] = float('inf')
            self.g[node] = float('inf')

        self.rhs[self.start] = 0
        self.U[self.start] = self.calculate_key(self.start)
        self.backup[self.start] = self.calculate_key(self.start)

    def _get_edge_cost(self, node_from, node_to):
        # TODO: This same function can be used in MDP (mdp3.py)
        # TODO: See lib.py
        edge_data = self.G.edges[node_from, node_to, 0]

        # If maxspeed is not given, use 30 as default. We assume that
        # bigger streets have a higher chance of being known and thus
        # contain maxspeed, while small streets might still be missing.
        if isinstance(edge_data.get('maxspeed'), list):
            maxspeed = 30
        else:
            maxspeed = int(edge_data.get('maxspeed', 30))

        return edge_data['length'] * 1000 / maxspeed

    def update_vertex(self, u):
        if u != self.start:
            nodes = list(self.G.predecessors(u))

            self.rhs[u] = min(
                    self.g[node] + self._get_edge_cost(node, u)
                    for node in nodes)
        if u in self.U:
            del self.U[u]

        if self.g[u] != self.rhs[u]:
            self.U[u] = self.calculate_key(u)
            self.backup[u] = self.calculate_key(u)

    def compute_shortest_path(self):
        # TODO Generally: Make sure to return False if g-value of goal node is
        # inf. That means we didn't find a path.
        # See get_shortest_path Exception as an idea. Not sure if that location
        # is appropriate btw, reconsider.
        print(self.U, min(self.U.values()), self.calculate_key(self.goal))
        while len(self.U) > 0 and \
                (min(self.U.values()) < self.calculate_key(self.goal) or
                    self.rhs[self.goal] != self.g[self.goal]):
            u = min(self.U, key=self.U.get)
            del self.U[u]

            if self.g[u] > self.rhs[u]:
                self.g[u] = self.rhs[u]
            else:
                self.g[u] = float('inf')
                self.update_vertex(u)

            for node in self.G.successors(u):
                self.update_vertex(node)

    def get_shortest_path(self):
        if self.g[self.goal] == float('inf'):
            raise Exception('No path found.')

        curr_node = self.goal

        visited = set({})

        col_nodes = []

        while curr_node != self.start:
            if not curr_node:
                break

            curr_min = float('inf')
            curr_min_node = None

            for node in self.G_projected.predecessors(curr_node):
                # TODO: Rename 'worth'
                worth = self.g[node] + aerial_dist(
                        self.G_projected.nodes[node],
                        self.G_projected.nodes[curr_node])

                if worth <= curr_min and node not in visited:
                    curr_min = worth
                    curr_min_node = node

            curr_node = curr_min_node

            # curr_node = out_edges[np.argmin(
            #     self.g[edge[1]] + self.aerial_dist(edge[1], curr_node)
            #     for edge in out_edges
            #     if edge[1] != last_node)][1]

            col_nodes.append(curr_node)
            visited.add(curr_node)

        return col_nodes


if __name__ == '__main__':
    with open('data/maxvorstadt.pickle', 'rb') as f:
        G = pickle.load(f)

    lpa = LPA_Star(G)
    lpa.init()
    lpa.compute_shortest_path()
    col_nodes = lpa.get_shortest_path()
    print(col_nodes)

    # print(lpa.backup)
    x = {}
    for k, v in lpa.backup.items():
        x[k] = v[0]

    nc, ns = get_node_properties(lpa.G_projected, col_nodes, x)
    for node in lpa.G_projected.nodes(data=True):
        lpa.G_projected.node[node[0]]['pos'] = (node[1]['x'], node[1]['y'])
    nx.draw_networkx(
            lpa.G_projected,
            nx.get_node_attributes(lpa.G_projected, 'pos'),
            node_size=ns,
            node_color=nc,
            node_zorder=2,
            with_labels=False)

    # labels = {}
    # for node in lpa.G_projected.nodes():
    #     labels[node] = lpa.heuristic_func(node)  # x.get(node, '')
    #     labels[node] = x.get(node, '')

    # nx.draw_networkx_labels(
    #         lpa.G_projected,
    #         nx.get_node_attributes(lpa.G_projected, 'pos'),
    #         labels=labels)

    plt.show()
