import pickle

import matplotlib.pyplot as plt
import networkx as nx

from mdp6 import MDP
from lib import get_time_to_drive
from lib import get_node_properties


with open('model5.pickle', 'rb') as f:
    mdp, V, Q = pickle.load(f)


policy = mdp.get_policy(V)
col_nodes = mdp.drive(policy)


print(policy)
print(col_nodes)
print('Minutes spent driving this route:', get_time_to_drive(col_nodes, mdp.G))


G2 = nx.MultiDiGraph()
for node in mdp.G.nodes.data():
    G2.add_node(node[0], pos=(node[1]['x'], node[1]['y']))

for s in mdp.S:
    if s in policy:
        G2.add_edge(*mdp.A[s][policy[s]])


G2.graph.update({
    'crs': mdp.G.graph['crs'],
    'name': mdp.G.graph['name']})


nc, ns = get_node_properties(G2, col_nodes, V, extra=mdp.close_nodes)
nx.draw_networkx(
        G2,
        nx.get_node_attributes(G2, 'pos'),
        node_size=ns,
        node_color=nc,
        node_zorder=2,
        with_labels=False)
edge_list = []
for edge in mdp.G.edges():
    if edge not in G2.edges():
        edge_list.append(edge)
nx.draw_networkx_edges(
        G2,
        nx.get_node_attributes(G2, 'pos'),
        edge_list,
        arrows=False)

plt.show()
