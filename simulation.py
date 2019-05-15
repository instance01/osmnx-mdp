import pickle
from timeit import default_timer as timer

import osmnx as ox

from mdp6 import MDP
from rtdp3 import RTDP
from lpa_star5 import LPA_Star


MAPS = {
    'Maxvorstadt': {
        'file': 'data/maxvorstadt.pickle',
        'place': 'Maxvorstadt, Munich, Germany'
    }
}


LOCATIONS = {
    'Maxvorstadt': [
        {
            'start': (48.1473062, 11.5632901),
            'goal': (48.1501497, 11.5809987),
            'metadata': {
                'id': 'RWUNI',
                'info': 'Richard-Wagner-Str to Geschwister-Scholl-Platz (University)',
            }
        }
    ]
}


def load_map_from_osm(place):
    ox.config(use_cache=True)
    G = ox.graph_from_place(place, network_type='drive', simplify=True)
    G = ox.add_edge_lengths(G)
    return G


def load_maps():
    for map_id, map_metadata in MAPS.items():
        try:
            with open(map_metadata['file'], 'rb') as f:
                G = pickle.load(f)
        except Exception as ex:
            print('Failed loading map pickle for %s.' % map_id, ex)
            print('Loading fresh map from OSM.')
            G = load_map_from_osm(map_metadata['place'])

            with open(map_metadata['file'], 'wb+') as f:
                pickle.dump(G, f)

        MAPS[map_id]['map'] = G

        for i, location in enumerate(LOCATIONS[map_id]):
            # TODO PEP-8
            LOCATIONS[map_id][i]['start'] = ox.utils.get_nearest_node(G, location['start'])
            LOCATIONS[map_id][i]['goal'] = ox.utils.get_nearest_node(G, location['goal'])


def run_simulation(algorithm, map_id, start, goal, diverge_policy):
    """Run a simulation on a map using a specific algorithm.

    Algorithm shall be an algorithm class such as RTDP or D* Lite that
    implements the functions:
        setup()
        solve()
        drive(policy, start, goal)

    start and goal shall be in the form of (lat, lon)
    diverge_policy shall be a function that returns for a given node and its
    options the next node.
    """
    start_time = timer()

    algorithm.setup(start, goal)
    policy = algorithm.solve()
    path = algorithm.drive(policy, start, goal)

    end_time = timer()

    # from lib import draw_value_graph
    # draw_value_graph(algorithm.G, path)

    print("Seconds for calculation:", end_time - start_time)


if __name__ == '__main__':
    load_maps()

    def diverge_policy(node, successors):
        # TODO
        return node

    for map_id in MAPS.keys():
        for location in LOCATIONS[map_id]:
            G = MAPS[map_id]['map']
            start = location['start']
            goal = location['goal']

            mdp = MDP(G)
            run_simulation(mdp, map_id, start, goal, diverge_policy)

            mdp = MDP(G)
            rtdp = RTDP(mdp)
            run_simulation(rtdp, map_id, start, goal, diverge_policy)

            lpa = LPA_Star(G)
            run_simulation(lpa, map_id, start, goal, diverge_policy)
