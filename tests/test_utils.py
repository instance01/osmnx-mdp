import networkx as nx


def _setup_mdp():
    G = nx.MultiDiGraph()

    G.add_node(1, x=0, y=0)
    G.add_node(2, x=2, y=0)
    G.add_node(3, x=3, y=1)
    G.add_node(4, x=3, y=1.2)
    G.add_node(5, x=0, y=2)
    G.add_node(6, x=4, y=1)

    G.add_edge(1, 2, length=6.)
    G.add_edge(1, 5, length=6.)
    G.add_edge(2, 3, length=4.)
    G.add_edge(2, 4, length=4.)
    G.add_edge(4, 6, length=4.)
    G.add_edge(6, 3, length=7.)

    G.graph.update({
        'crs': {'init': 'epsg:4326'},
        'name': 'Maxvorstadt, Munich, Germany'
    })

    return G
