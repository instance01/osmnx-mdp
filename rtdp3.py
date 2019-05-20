import pickle

import numpy as np

from algorithm import Algorithm
from lib import aerial_dist
from lib import get_time_to_drive
from lib import draw_value_graph
from mdp6 import MDP


class RTDP(Algorithm):
    def __init__(self, mdp):
        self.mdp = mdp
        self.H = {}
        self.h = {}

    def setup(self, start, goal):
        self.start = start
        self.goal = goal
        self.G = self.mdp.G
        self.Q = {}

        goal = self.G.nodes.data()[self.goal]

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
        # TODO: Stop criterion is missing, this is vanilla RTDP.
        # TODO: Add LRTDP, BRTDP
        for _ in range(n):
            path = self._run_trial()
        return path

    def solve(self):
        return self.run_trials()

    def drive(self, path, diverge_policy={}):
        # TODO: The below is an own implementation of RTDP as a replanner.
        # There's no publication for this as far as I know, so take with a
        # grain of salt.
        # Options:
        # * Try using old Q values (but if we diverge too far, the states
        #   don't have Q values)
        # * Replan from current node, so run trials again
        visited = set()

        path_lookup = set(path)
        path = iter(path)

        nodes = [self.start]
        curr_node = next(path)
        while curr_node != self.goal:
            if curr_node not in path_lookup:
                # Replan
                self.start = curr_node
                # TODO: Reuse H or not?
                self.H = {}

                path = self.run_trials()

                path_lookup = set(path)
                path = iter(path)

                # Path returned by run_trials always contains start node
                # too. We don't need it.
                next(path)

            diverged_node = diverge_policy.get(curr_node, None)
            next_curr_path = next(path)
            # TODO: PEP-8
            going_backwards = diverged_node in visited or diverged_node in path_lookup
            if diverged_node is None or going_backwards:
                curr_node = next_curr_path
            else:
                curr_node = diverged_node
                visited.add(diverged_node)

            nodes.append(curr_node)

        return nodes


if __name__ == '__main__':
    with open('data/maxvorstadt.pickle', 'rb') as f:
        G = pickle.load(f)

    mdp = MDP(G)
    mdp.setup(246878841, 372796487)

    rtdp = RTDP(mdp)
    rtdp.setup(246878841, 372796487)
    path = rtdp.run_trials()

    time_to_drive = get_time_to_drive(path, rtdp.G)  # Minutes
    print('Minutes spent driving this route:', time_to_drive)

    draw_value_graph(rtdp.G, path, rtdp.H)
