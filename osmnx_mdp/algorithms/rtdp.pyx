import pickle

import numpy as np

from osmnx_mdp.lib cimport aerial_dist
from osmnx_mdp.lib cimport get_time_to_drive
from osmnx_mdp.lib cimport draw_value_graph

from osmnx_mdp.algorithms.mdp cimport MDP


cdef class RTDP(osmnx_mdp.algorithms.algorithm.Algorithm):
    def __init__(self, mdp):
        self.mdp = mdp
        self.H = {}
        self.h = {}

    cdef setup(self, long start, long goal):
        self.start = start
        self.goal = goal
        self.G = self.mdp.G
        self.Q = {}

        goal_node = self.G.nodes.data()[self.goal]

        for node in self.G.nodes.data():
            self.h[node[0]] = aerial_dist(node[1], goal_node)
            # TODO: We can just use 0-heuristic too?
            # self.h[node[0]] = 0
            self.Q[node[0]] = {action: 0 for action in self.mdp.A[node[0]]}

    cdef _select_next_state(self, state, action):
        """Select next state probabilistically.
        """
        outcomes = self.mdp.P[state][action]
        a = [outcome[0] for outcome in outcomes]
        p = [outcome[1] for outcome in outcomes]
        return np.random.choice(a, 1, p)[0]

    cdef _run_trial(self):
        state = self.start

        visited = [state]

        while state != self.goal:
            for action in self.mdp.A[state]:
                node_to = action[1]
                cost = self.mdp.C[(state, node_to)]
                future_cost = sum([
                        x[1] * self.H.get(x[0], self.h[x[0]])
                        for x in self.mdp.P[state][action]])
                self.Q[state][action] = cost + future_cost

            if not self.Q[state]:
                break

            best_action = min(self.Q[state], key=self.Q[state].get)
            self.H[state] = self.Q[state][best_action]

            state = self._select_next_state(state, best_action)
            visited.append(state)

        return visited

    cdef run_trials(self, n=1000):
        # TODO: Stop criterion is missing, this is vanilla RTDP.
        for _ in range(n):
            path = self._run_trial()
        return path

    cdef solve(self):
        return self.run_trials()

    cdef drive(self, path, diverge_policy):
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
                try:
                    self.start = curr_node
                except OverflowError:
                    print(curr_node, self.start)
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
