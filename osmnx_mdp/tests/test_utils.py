import networkx as nx


def _setup_mdp(remove_dead_ends=False):
    G = nx.MultiDiGraph()

    G.add_node(1, x=0, y=0, lat=0, lon=0)
    G.add_node(2, x=2, y=0, lat=2, lon=0)
    G.add_node(3, x=4, y=1, lat=3, lon=1)
    G.add_node(4, x=4, y=1.2, lat=3, lon=1.2)
    G.add_node(5, x=0, y=2, lat=0, lon=2)
    G.add_node(6, x=8, y=1, lat=4, lon=1)

    G.add_edge(1, 2, length=6.)
    G.add_edge(1, 5, length=6.)
    G.add_edge(2, 3, length=4.)
    G.add_edge(2, 4, length=4.)
    G.add_edge(3, 6, length=4.)
    G.add_edge(4, 6, length=9.)
    G.add_edge(6, 3, length=7.)
    G.add_edge(3, 2, length=4.)

    if remove_dead_ends:
        G.remove_node(5)

    G.graph.update({
        'crs': {'init': 'epsg:4326'},
        'name': 'Maxvorstadt, Munich, Germany'
    })

    return G


def _get_initialized_mdp_attrs():
    A = {
            1: [(1, 2)],
            2: [(2, 3), (2, 4)],
            3: [(3, 6), (3, 2)],
            4: [(4, 6)],
            6: [(6, 3), (6, 6)]
        }
    C = {
            (1, 2): 0.0002,
            (2, 3): 0.00013333333333333333,
            (2, 4): 0.00013333333333333333,
            (3, 2): 0.00013333333333333334,
            (3, 6): 0.00013333333333333334,
            (4, 6): 0.0003,
            (6, 3): 0.00023333333333333334,
            (6, 6): 0
        }
    P = {
            1: {(1, 2): [(2, 1.0)]},
            2: {(2, 3): [(3, 0.9), (4, 0.1)], (2, 4): [(4, 0.9), (3, 0.1)]},
            3: {(3, 6): [(6, 1.0)], (3, 2): [(2, 1.0)]},
            4: {(4, 6): [(6, 1.0)]},
            6: {(6, 3): [(3, 1.0)], (6, 6): [(6, 1.0)]}
        }
    S = set([1, 2, 3, 4, 6])

    return A, C, P, S


def _get_solved_attrs():
    V_want = {
        1: 0.00048333333333333334,
        2: 0.00028333333333333335,
        3: 0.00013333333333333334,
        4: 0.0003,
        6: 0.0
    }
    Q_want = {
        1: {(1, 2): 0.00048333333333333334},
        2: {(2, 3): 0.00028333333333333335, (2, 4): 0.0004166666666666667},
        3: {(3, 6): 0.00013333333333333334, (3, 2): 0.0004166666666666667},
        4: {(4, 6): 0.0003},
        6: {(6, 3): 0.00036666666666666667, (6, 6): 0.0}
    }

    return V_want, Q_want


def _get_angle_updated_P():
    P = {
        1: {(1, 2): [(2, 1.0)]},
        2: {(2, 3): [(3, 0.9), (4, 0.1)], (2, 4): [(4, 0.9), (3, 0.1)]},
        3: {(3, 6): [(6, 1.0)], (3, 2): [(2, 1.0)]},
        4: {(4, 6): [(6, 1.0)]},
        6: {(6, 3): [(3, 1.0)], (6, 6): [(6, 1.0)]}
    }
    return P
