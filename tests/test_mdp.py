import unittest
from unittest.mock import patch

import networkx as nx

import mdp6
from test_utils import _setup_mdp


class MDPTest(unittest.TestCase):
    def _get_initialized_mdp_attrs(self):
        A = {
                1: [(1, 2), (1, 5), (1, 6)],
                2: [(2, 3), (2, 4), (2, 6)],
                3: [(3, 6)],
                4: [(4, 6), (4, 6)],
                5: [(5, 6)],
                6: [(6, 3), (6, 6)]
            }
        C = {
                (1, 2): 0.0002,
                (1, 5): 0.0002,
                (1, 6): 100000.,
                (2, 3): 0.00013333333333333333,
                (2, 4): 0.00013333333333333333,
                (2, 6): 100000.,
                (3, 6): 100000.,
                (4, 6): 0.00013333333333333333,
                (5, 6): 100000.,
                (6, 3): 0.00023333333333333334,
                (6, 6): 100000.
            }
        P = {
                1: {(1, 2): [(2, 1.0)], (1, 5): [(5, 1.0)], (1, 6): [(6, 1.0)]},
                2: {(2, 3): [(3, 1.0)], (2, 4): [(4, 1.0)], (2, 6): [(6, 1.0)]},
                3: {(3, 6): [(6, 1.0)]},
                4: {(4, 6): [(6, 1.0)]},
                5: {(5, 6): [(6, 1.0)]},
                6: {(6, 3): [(3, 1.0)], (6, 6): [(6, 1.0)]}
            }
        S = set([1, 2, 3, 4, 5, 6])

        return A, C, P, S

    def _get_solved_attrs(self):
        V_want = {
            1: 0.00046666666666666666,
            2: 0.00026666666666666666,
            3: 100000.0,
            4: 0.00013333333333333333,
            5: 100000.0,
            6: 0.0
        }

        Q_want = {
            1: {(1, 2): 0.00046666666666666666, (1, 5): 100000.0002, (1, 6): 100000.0},
            2: {(2, 3): 100000.00013333333333, (2, 4): 0.00026666666666666666, (2, 6): 100000.0},
            3: {(3, 6): 100000.0},
            4: {(4, 6): 0.00013333333333333333},
            5: {(5, 6): 100000.0},
            6: {(6, 3): 100000.00023333333334, (6, 6): 0.0}
        }

        return V_want, Q_want

    def _get_angle_updated_P(self):
        P = {
                1: {(1, 2): [(2, 1.0)], (1, 5): [(5, 1.0)], (1, 6): [(6, 1.0)]},
                2: {(2, 3): [(3, 0.9), (4, 0.1)], (2, 4): [(4, 0.9), (3, 0.1)], (2, 6): [(6, 1.0)]},
                3: {(3, 6): [(6, 1.0)]},
                4: {(4, 6): [(6, 1.0)]},
                5: {(5, 6): [(6, 1.0)]},
                6: {(6, 3): [(3, 1.0)], (6, 6): [(6, 1.0)]}
            }
        return P

    def test_get_angle(self):
        have = mdp6.get_angle((9, 40), (50, 11), (5, 3))
        want = 73.74922691742808
        self.assertEqual(have, want)

    def test_add_costly_jump_to_goal(self):
        mdp = mdp6.MDP(_setup_mdp())
        mdp.add_costly_jump_to_goal(4)
        self.assertEqual(mdp.A, {4: [(4, 372796487)]})
        self.assertEqual(mdp.P, {4: {(4, 372796487): [(372796487, 1.0)]}})
        self.assertEqual(mdp.C, {(4, 372796487): 100000.0})

    def test_mdp_setup(self):
        mdp = mdp6.MDP(_setup_mdp())
        mdp.goal = 6
        mdp._setup()

        A, C, P, S = self._get_initialized_mdp_attrs()

        self.assertEqual(mdp.A, A)
        self.assertEqual(mdp.C, C)
        self.assertEqual(mdp.P, P)
        self.assertEqual(mdp.S, S)

    @patch('osmnx.project_graph')
    def test_update_uncertain_intersections(self, project_graph_mock):
        G = _setup_mdp()
        project_graph_mock.return_value = G
        mdp = mdp6.MDP(G)
        mdp.goal = 6
        mdp._setup()
        A, C, P, S = self._get_initialized_mdp_attrs()
        mdp.A = A
        mdp.C = C
        mdp.P = P
        mdp.S = S
        angles = mdp.update_uncertain_intersections()
        self.assertEqual(angles, [2])
        P = self._get_angle_updated_P()
        self.assertEqual(mdp.P, P)

        # We're coming from node 3, end up in 2, with a kind of sharp T-shaped
        # intersection.
        # This is NOT a critical intersection.
        #     2
        #   / | \
        #  /  |  \
        # 1   3   4
        G = nx.MultiDiGraph()

        G.add_node(1, x=0, y=0)
        G.add_node(2, x=2, y=5)
        G.add_node(3, x=2, y=0)
        G.add_node(4, x=2.6, y=0)

        G.add_edge(2, 1, length=6.)
        G.add_edge(2, 4, length=6.)
        G.add_edge(3, 2, length=4.)

        project_graph_mock.return_value = G
        mdp = mdp6.MDP(G)
        mdp.goal = 6
        mdp._setup()
        angles = mdp.update_uncertain_intersections()
        self.assertEqual(angles, [])

        # We're coming from node 1.
        # This is a critical intersection.
        #     2
        #   / | \
        #  /  |  \
        # 1   3   4
        G = nx.MultiDiGraph()

        G.add_node(1, x=0, y=0)
        G.add_node(2, x=2, y=5)
        G.add_node(3, x=2, y=0)
        G.add_node(4, x=2.6, y=0)

        G.add_edge(2, 3, length=6.)
        G.add_edge(2, 4, length=6.)
        G.add_edge(1, 2, length=4.)

        project_graph_mock.return_value = G
        mdp = mdp6.MDP(G)
        mdp.goal = 6
        mdp._setup()
        angles = mdp.update_uncertain_intersections()
        self.assertEqual(angles, [2])

        # We're coming from node 3 and end up in a T-shaped intersection.
        # This is NOT a critical intersection.
        # 1--2--4
        #    |
        #    3
        G = nx.MultiDiGraph()

        G.add_node(1, x=0, y=4)
        G.add_node(2, x=2, y=4)
        G.add_node(3, x=2, y=0)
        G.add_node(4, x=4, y=4)

        G.add_edge(2, 1, length=6.)
        G.add_edge(2, 4, length=6.)
        G.add_edge(3, 2, length=4.)

        project_graph_mock.return_value = G
        mdp = mdp6.MDP(G)
        mdp.goal = 6
        mdp._setup()
        angles = mdp.update_uncertain_intersections()
        self.assertEqual(angles, [])

    @patch('osmnx.project_graph')
    def test_solve_value_iteration(self, project_graph_mock):
        G = _setup_mdp()
        project_graph_mock.return_value = G
        mdp = mdp6.MDP(G)
        mdp.goal = 6
        mdp.A, mdp.C, mdp.P, mdp.S = self._get_initialized_mdp_attrs()

        mdp.C[6, 6] = 0.

        V, Q = mdp.solve_value_iteration()

        V_want, Q_want = self._get_solved_attrs()

        self.assertEqual(V, V_want)
        self.assertEqual(Q, Q_want)

    def test_get_policy(self):
        mdp = mdp6.MDP(_setup_mdp())
        mdp.goal = 6
        mdp.A, mdp.C, mdp.P, mdp.S = self._get_initialized_mdp_attrs()
        mdp.P = self._get_angle_updated_P()

        V_want, Q_want = self._get_solved_attrs()

        want = {1: 0, 2: 1, 3: 0, 4: 0, 5: 0, 6: 1}
        have = mdp.get_policy(V_want)

        self.assertEqual(want, have)

    def test_make_edge_uncertain(self):
        mdp = mdp6.MDP(_setup_mdp())
        temp_P = {}
        mdp._make_edge_uncertain(temp_P, (1, 2), 3)
        self.assertEqual(temp_P, {(1, 2): [(2, 0.9), (3, 0.1)]})

    @patch('osmnx.project_graph')
    def test_remove_zero_cost_loops(self, project_graph_mock):
        G = nx.MultiDiGraph()

        G.add_node(1, x=0, y=0)
        G.add_node(2, x=2, y=5)
        G.add_node(3, x=2, y=0)

        G.add_edge(2, 3, length=6.)
        G.add_edge(3, 3, length=6.)
        G.add_edge(2, 1, length=4.)

        project_graph_mock.return_value = G
        mdp = mdp6.MDP(G)

        mdp.remove_zero_cost_loops()

        self.assertEqual(list(mdp.G.edges()), [(2, 3), (2, 1)])

    def test_make_goal_self_absorbing(self):
        mdp = mdp6.MDP(_setup_mdp())
        mdp.goal = 1
        mdp.make_goal_self_absorbing()
        self.assertEqual(mdp.A, {1: [(1, 1)]})
        self.assertEqual(mdp.P, {1: {(1, 1): [(1, 1.0)]}})
        self.assertEqual(mdp.C, {(1, 1): 0})

    def test_remove_dead_ends(self):
        # TODO Test
        pass

    def test_get_Q_value(self):
        # TODO Test
        pass

    def test_drive(self):
        # TODO Test
        pass


if __name__ == '__main__':
    unittest.main()
