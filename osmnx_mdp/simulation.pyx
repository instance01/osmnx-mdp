# cython: language_level=3
import multiprocessing
import random
import pickle
from timeit import default_timer as timer

import numpy as np
import osmnx as ox

from osmnx_mdp.lib cimport draw_value_graph
from osmnx_mdp.lib cimport remove_dead_ends
from osmnx_mdp.lib cimport remove_zero_cost_loops
from osmnx_mdp.lib cimport get_time_to_drive
from osmnx_mdp.algorithms.mdp cimport MDP
from osmnx_mdp.algorithms.brtdp cimport BRTDP
from osmnx_mdp.algorithms.brtdp_replan cimport BRTDP_REPLAN
from osmnx_mdp.algorithms.dstar_lite cimport DStar_Lite
from osmnx_mdp.algorithms.algorithm cimport Algorithm


# For each simulation this config is adapted.
# Algorithm configs always map 'bytestring -> double' due to static typing in
# C++.
CONFIG = {
    'processes': 1,
    'diverge_policy': 'random',  # Options: random, model
    'random_policy_percent': .2,
    'include_traffic_signals_in_metric': True,
    'MDP': {
        # Cython needs byte strings
        b'max_angle': 35,
        b'max_length': 200,
        b'edge_uncertainty': .2
    },
    'BRTDP': {
        b'alpha': 1e-20,
        b'tau': 100,
        b'heuristic': 1, # Options: 0 for Dijkstra, 1 for aerial distance
        b'heuristic_max_speed': 200 # In km/h
    },
    'BRTDP_REPLAN': {
        b'alpha': 1e-20,
        b'tau': 100,
        b'beta': .02,
        b'always_replan': 1
    },
    'DStar_Lite': {
        b'heuristic': 1, # Options: 0 for Dijkstra, 1 for aerial distance
        b'heuristic_max_speed': 200 # In km/h
    }
}


MAPS = {
    'Maxvorstadt': {
        'type': 'place',
        'file': 'data/maxvorstadt.pickle',
        'place': 'Maxvorstadt, Munich, Germany'
    },
    'Munich': {
        'type': 'place',
        'file': 'data/munich.pickle',
        'place': 'Munich, Germany'
    },
    'Berlin': {
        'type': 'place',
        'file': 'data/berlin.pickle',
        'place': 'Berlin, Germany',
        'which_result': 2
    },
    'Köln': {
        'type': 'place',
        'file': 'data/koeln.pickle',
        'place': 'Köln, Germany'
    },
    #'Munich_Environment': {
    #    'type': 'bbox',
    #    'file': 'data/munich_environment.pickle',
    #    'north': '48.332197',
    #    'west': '11.361083',
    #    'south': '48.003655',
    #    'east': '11.764612'
    #},
    'Starnberg_Environment': {
        'type': 'bbox',
        'file': 'data/starnberg_environment.pickle',
        'north': '48.032605',
        'west': '11.247803',
        'south': '47.809321',
        'east': '11.502211'
    },
    'Bavaria': {
        'type': 'place',
        'file': 'data/bavaria.pickle',
        'place': 'Bavaria, Germany'
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
    ],
    'Munich': [
        {
            'start': (48.164821, 11.3383748),
            'goal': (48.139404, 11.6708573),
            'metadata': {
                'id': 'AURIMUN',
                'info': 'Aubing to Riem through Munich',
            }
        }
    ],
    'Berlin': [
        {
            'start': (52.4172733, 13.1389813),
            'goal': (52.5667931, 13.5228932),
            'metadata': {
                'id': 'BERLINLONG',
                'info': ''
            }
        }, {
            'start': (52.5890102, 13.2817751),
            'goal': (52.4260165, 13.5340521),
            'metadata': {
                'id': 'BERLINLONG1',
                'info': ''
            }
        }, {
            'start': (52.5878025, 13.2854392),
            'goal': (52.469358, 13.5531717),
            'metadata': {
                'id': 'BERLINLONG2',
                'info': ''
            }
        }
    ],
    'Köln': [
        {
            'start': (50.9384349, 6.826471),
            'goal': (50.9313467, 6.9764982),
            'metadata': {
                'id': 'KOELNLONG',
                'info': ''
            }
        }
    ],
    'Munich_Environment': [
        # TODO
    ],
    'Starnberg_Environment': [
        {
            'start': (47.9427064, 11.2579095),
            'goal': (47.9648109, 11.4076497),
            'metadata': {
                'id': 'StarnbergLONG',
                'info': ''
            }
        }
    ],
    'Bavaria': [
        {
            'start': (47.7289566, 10.3077225),
            'goal': (49.9361796, 11.7199061),
            'metadata': {
                'id': 'BavariaLONG',
                'info': 'Diagonally through Bavaria'
            }
        }
    ]
}


def load_map_from_osm(map_metadata):
    ox.config(use_cache=True)
    if map_metadata['type'] == 'place':
        G = ox.graph_from_place(
                map_metadata['place'],
                network_type='drive',
                which_result=map_metadata.get('which_result', 1),
                simplify=True)
    else:
        G = ox.graph_from_bbox(
                float(map_metadata['north']),
                float(map_metadata['south']),
                float(map_metadata['east']),
                float(map_metadata['west']),
                network_type='drive',
                simplify=True)
    G = ox.add_edge_lengths(G)
    return G


def _update_locs(G, map_id):
    # TODO: Weird utility function
    locs = []
    for i, location in enumerate(LOCATIONS[map_id]):
        print(map_id, i, location)
        # TODO PEP-8
        LOCATIONS[map_id][i]['start'] = ox.utils.get_nearest_node(G, location['start'])
        LOCATIONS[map_id][i]['goal'] = ox.utils.get_nearest_node(G, location['goal'])
    locs = LOCATIONS[map_id]
    return locs


def load_maps():
    for map_id, map_metadata in MAPS.items():
        try:
            with open(map_metadata['file'], 'rb') as f:
                locs, G = pickle.load(f)
        except Exception as ex:
            print('Failed loading map pickle for %s.' % map_id, ex)
            print('Loading fresh map from OSM.')
            G = load_map_from_osm(map_metadata)

            locs = _update_locs(G, map_id)

            print('Projecting graph. This might take a while..')
            G = ox.project_graph(G)

            with open(map_metadata['file'], 'wb+') as f:
                pickle.dump((locs, G), f)

        MAPS[map_id]['map'] = G
        LOCATIONS[map_id] = locs


def run_simulation(
        Algorithm algorithm,
        map_id,
        location_id,
        start,
        goal,
        diverge_policy,
        debug=False):
    """Run a simulation on a map using a specific algorithm.

    Algorithm shall be an algorithm class such as RTDP or D* Lite that
    implements the functions:
        setup()
        solve()
        drive(policy, diverge_policy)

    start and goal shall be in the form of (lat, lon)
    diverge_policy shall be a function in the form diverge_policy(G, node)
    and return for a given node and its options (successors) the next node.
    """
    algorithm_name = algorithm.__class__.__name__
    print("Running %s on map %s." % (algorithm_name, map_id))
    start_time = timer()

    algorithm.setup(start, goal, CONFIG[algorithm_name])
    policy = algorithm.solve()
    path = algorithm.drive(policy, diverge_policy)

    end_time = timer()

    total_time = end_time - start_time
    drive_time = get_time_to_drive(
            path,
            MAPS[map_id]['map'],
            CONFIG['include_traffic_signals_in_metric'])

    if debug:
        draw_value_graph(MAPS[map_id]['map'], path)

    result = {
        'algorithm': algorithm_name,
        'id': location_id,
        'calculation_time': total_time,
        'drive_time': drive_time,
        'n_nodes': MAPS[map_id]['map'].number_of_nodes(),
        'path': path,
        'diverge_policy': diverge_policy,
        'iterations': algorithm.iterations
    }

    print("[%s, %s, %s] Seconds for calculation: %f" % (
        algorithm_name, map_id, location_id, total_time)
    )
    return result


def gen_diverge_policy(G, func):
    """Generate a diverge policy based on a general function.
    """
    policy = {}

    for node in G.nodes():
        if func(node):
            # Make sure successors are not already in diverge_policy, else we
            # might get a loop (two nodes diverging to each other in a loop).
            successors = [
                succ for succ in G.successors(node)
                if succ not in policy
            ]
            if len(successors) == 0:
                continue
            policy[node] = np.random.choice(successors, 1)[0]

    return policy


def gen_random_diverge_policy(G, density=.2):
    """Generate a divergence policy for all nodes:
    Diverge at roughly $(density) percent of nodes.
    We diverge by taking a random successor of the node.
    So this could also lie on the real path.
    """
    func = lambda _: random.random() < density
    return gen_diverge_policy(G, func)


def gen_diverge_policy_with_uncertain_nodes(G, uncertain_nodes):
    """Generate a divergence policy for all nodes based on uncertain nodes from
    the MDP model.
    Right now uncertainty in the model is based on low angle intersections and
    close intersections. See more in the MDP class.
    """
    func = lambda node: node in uncertain_nodes
    return gen_diverge_policy(G, func)


# TODO Rename
def run(map_id, location):
    curr_result = []

    location_id = location['metadata']['id']

    start = location['start']
    goal = location['goal']

    G = MAPS[map_id]['map']

    print('Preprocessing graph..')
    remove_zero_cost_loops(G)
    remove_dead_ends(G, goal)

    print('filtered n_nodes/goal:', G.number_of_nodes(), goal)

    if CONFIG['diverge_policy'] == 'random':
        diverge_policy = gen_random_diverge_policy(G, CONFIG['random_policy_percent'])
    else:
        mdp = MDP(G)
        mdp.setup(start, goal, CONFIG['MDP'])
        diverge_policy = gen_diverge_policy_with_uncertain_nodes(G, mdp.uncertain_nodes)

    mdp = MDP(G)
    curr_result.append(run_simulation(mdp, map_id, location_id, start, goal, diverge_policy))

    # mdp = MDP(G)
    # mdp.setup(start, goal, CONFIG['MDP'])
    # brtdp = BRTDP_REPLAN(mdp)
    # curr_result.append(run_simulation(brtdp, map_id, location_id, start, goal, diverge_policy))

    mdp = MDP(G)
    mdp.setup(start, goal, CONFIG['MDP'])
    brtdp = BRTDP(mdp)
    curr_result.append(run_simulation(brtdp, map_id, location_id, start, goal, diverge_policy))

    dstar = DStar_Lite(G)
    curr_result.append(run_simulation(dstar, map_id, location_id, start, goal, diverge_policy))

    return curr_result


def run_simulations(iterations=1):
    start_time = timer()

    load_maps()
    
    results = []

    # Target machine has 4 cores.
    # Let's use them to speed up the simulation significantly.
    # It's important to keep in mind that this might falsify the results, if
    # not executed correcty.
    # For example, we cannot have more than 4 workers in the pool.
    # If we have 5, two might fight for resources on one core. They will be
    # slower than they would be if they had a full core for themselves.

    # maxtasksperchild=1 to make sure resources are cleaned up completely after
    # a worker finishes. So memory access to the hashmaps isn't different in
    # subsequent runs for some algorithms.
    pool = multiprocessing.Pool(processes=CONFIG["processes"], maxtasksperchild=1)

    for _ in range(iterations):
        # TODO: Rename all this

        curr_result = []
        async_results = []

        for map_id in MAPS.keys():
            for location in LOCATIONS[map_id]:
                async_results.append(pool.apply_async(run, (map_id, location)))

        for async_result in async_results:
            curr_result.extend(async_result.get())

        results.append(curr_result)

        # Save results to simulation.pickle after each iteration so if it should
        # ever crash, we have partial results.
        # The point is, they're only improving slightly per iteration, so these
        # 'partial' results would be very usable.
        print('Saving temporary results to simulation.pickle..')
        with open('simulation.pickle', 'wb+') as f:
            pickle.dump(results, f)

    pool.close()
    pool.join()
    print('Total time for simulation:', timer() - start_time)


if __name__ == '__main__':
    run_simulations()
