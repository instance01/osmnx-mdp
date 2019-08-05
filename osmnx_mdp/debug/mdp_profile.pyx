# cython: language_level=3
import cProfile
import pstats
import pickle

from osmnx_mdp.algorithms.mdp cimport MDP
from osmnx_mdp.algorithms.dstar_lite cimport DStar_Lite
from osmnx_mdp.lib cimport remove_dead_ends
from osmnx_mdp.lib cimport remove_zero_cost_loops

#from osmnx_mdp.algorithms.mdp cimport MDP
from osmnx_mdp.lib cimport get_time_to_drive
from osmnx_mdp.lib cimport get_node_properties



PROFILE_FILE = 'mdp1.profile'


with open('data/munich.pickle', 'rb') as f:
    locs, G = pickle.load(f)


start = locs[0]['start']
goal = locs[0]['goal']

remove_zero_cost_loops(G)
remove_dead_ends(G, goal)


def run():
    cfg =  {
        'max_angle': 35,
        'max_length': 200,
        'edge_uncertainty': .2
    }
    mdp = MDP(G)
    mdp.setup(start, goal, cfg)
    mdp.solve_value_iteration(gamma=1., max_iter=1000)

    #dstar = DStar_Lite(G)
    #dstar.setup(start, goal)
    #dstar.compute_shortest_path()
    #col_nodes = list(dstar.drive({}, {}))
    #print(col_nodes)


cProfile.runctx("run()", globals(), locals(), PROFILE_FILE)


out = pstats.Stats(PROFILE_FILE)

print('sort by cumulative (the function itself plus all child functions called inside)')
out.sort_stats('cumulative').print_stats(20)

print('sort by total time (only the function itself not its childrens)')
out.sort_stats('time').print_stats(20)
