import pickle
from algorithms.dstar_lite import DStar_Lite
from lib import get_node_properties
import networkx as nx
import matplotlib.pyplot as plt


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
