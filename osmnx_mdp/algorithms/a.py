import pickle
from osmnx_mdp.lib import remove_dead_ends
from osmnx_mdp.algorithms.mdp import MDP


with open('../data/maxvorstadt.pickle', 'rb') as f:
    G = pickle.load(f)

remove_dead_ends(G, 372796487)

mdp = MDP(G)
mdp.start = 1
mdp.setup(246878841, 372796487)
V, Q = mdp.solve_value_iteration()
