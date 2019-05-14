import pickle

import numpy as np

from lib import aerial_dist
from lib import get_time_to_drive
from lib import draw_value_graph
from mdp6 import MDP


class RTDP(object):
    def __init__(self, mdp):
        self.mdp = mdp
        self.H = {}
        self.h = {}
        self.start = 246878841
        self.goal = 372796487

    def init(self):
        self.G = self.mdp.G
        goal = self.G.nodes.data()[self.goal]
        self.Q = {}
        for node in self.G.nodes.data():
            # TODO: Below comment is outdated, multiplier not needed anymore.
            # The heuristic has to be admissible. Apparently *10 or even no
            # multiplier doesn't cut it. 100 is too much, we literally descend
            # into a greedy best-only search. But 20 seems to be a good one.
            # It seems *20 is best.

            self.h[node[0]] = aerial_dist(node[1], goal)
            # TODO: We can just use 0-heuristic too?
            # self.h[node[0]] = 0
            self.Q[node[0]] = {action: 0 for action in self.mdp.A[node[0]]}

    def _select_next_state(self, state, action):
        """Select next state probabilistically.
        """
        outcomes = self.mdp.P[state][action]
        a = [outcome[0] for outcome in outcomes]
        p = [outcome[1] for outcome in outcomes]
        return np.random.choice(a, 1, p)[0]

    def _run_trial(self):
        state = self.start

        visited = [state]

        while state != self.goal:
            for action in self.mdp.A[state]:
                node_to = action[1]
                cost = self.mdp.C[(state, node_to)]
                future_cost = sum(
                        x[1] * self.H.get(x[0], self.h[x[0]])
                        for x in self.mdp.P[state][action])
                self.Q[state][action] = cost + future_cost

            if not self.Q[state]:
                break

            best_action = min(self.Q[state], key=self.Q[state].get)
            self.H[state] = self.Q[state][best_action]

            state = self._select_next_state(state, best_action)
            visited.append(state)

        return visited

    def run_trials(self, n=1000):
        for _ in range(n):
            path = self._run_trial()
        return path


if __name__ == '__main__':
    with open('maxvorstadt2.pickle', 'rb') as f:
        G = pickle.load(f)

    mdp = MDP(G)
    mdp.remove_zero_cost_loops()
    mdp.remove_dead_ends()
    mdp.setup()
    mdp.make_goal_self_absorbing()
    mdp.angle_nodes = mdp.update_uncertain_intersections()

    rtdp = RTDP(mdp)
    rtdp.init()
    path = rtdp.run_trials()

    time_to_drive = get_time_to_drive(path, rtdp.G)  # Minutes
    print('Minutes spent driving this route:', time_to_drive)

    draw_value_graph(rtdp.G, path, rtdp.H)
