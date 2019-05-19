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
                4: [(4, 6)],
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
        mdp.goal = 6
        mdp._add_costly_jump_to_goal(4)
        self.assertEqual(mdp.A, {})
        self.assertEqual(mdp.P, {})
        self.assertEqual(mdp.C, {})

        mdp._add_costly_jump_to_goal(1)
        self.assertEqual(mdp.A, {1: [(1, 6)]})
        self.assertEqual(mdp.P, {1: {(1, 6): [(6, 1.0)]}})
        self.assertEqual(mdp.C, {(1, 6): 100000.0})

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
    def test_make_low_angle_intersections_uncertain(self, project_graph_mock):
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
        angles = mdp.make_low_angle_intersections_uncertain()
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
        angles = mdp.make_low_angle_intersections_uncertain()
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
        angles = mdp.make_low_angle_intersections_uncertain()
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
        angles = mdp.make_low_angle_intersections_uncertain()
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

    @patch('osmnx.project_graph')
    def test_remove_dead_ends(self, project_graph_mock):
        #  1 - 2 - 8
        #  |   |
        #  4 - 3
        #  |
        #  5 - 7
        #  |
        #  6
        G = nx.MultiDiGraph()
        G.add_edge(1, 2)
        G.add_edge(2, 3)
        G.add_edge(3, 4)
        G.add_edge(4, 1)
        G.add_edge(4, 5)
        G.add_edge(5, 6)
        G.add_edge(5, 7)
        G.add_edge(2, 8)

        project_graph_mock.return_value = G
        mdp = mdp6.MDP(G)
        mdp.goal = 8
        mdp.remove_dead_ends()

        self.assertEqual(list(mdp.G.nodes()), [1, 2, 3, 4, 8])

        #  1 - 2
        #  |   |
        #  4 - 3
        #  |
        #  5<->6
        G = nx.MultiDiGraph()
        G.add_edge(1, 2)
        G.add_edge(2, 3)
        G.add_edge(3, 4)
        G.add_edge(4, 1)
        G.add_edge(4, 5)
        G.add_edge(5, 6)
        G.add_edge(6, 5)

        project_graph_mock.return_value = G
        mdp = mdp6.MDP(G)
        mdp.goal = 3
        mdp.remove_dead_ends()

        self.assertEqual(list(mdp.G.nodes()), [1, 2, 3, 4])

        #  1 - 2 - 9
        #  |   |
        #  4 - 3
        #  |
        #  5<->6<->8
        #  |
        #  7
        G = nx.MultiDiGraph()
        G.add_edge(1, 2)
        G.add_edge(2, 3)
        G.add_edge(3, 4)
        G.add_edge(4, 1)
        G.add_edge(4, 5)
        G.add_edge(5, 6)
        G.add_edge(6, 5)
        G.add_edge(6, 8)
        G.add_edge(8, 6)
        G.add_edge(5, 7)
        G.add_edge(2, 9)

        project_graph_mock.return_value = G
        mdp = mdp6.MDP(G)
        mdp.goal = 3
        mdp.remove_dead_ends()

        self.assertEqual(list(mdp.G.nodes()), [1, 2, 3, 4])

    @patch('osmnx.project_graph')
    def test_get_normal_intersection(self, project_graph_mock):
        G = nx.MultiDiGraph()

        G.add_node(1, x=2, y=0)
        G.add_node(2, x=2, y=2)
        G.add_node(3, x=0, y=2)
        G.add_node(4, x=4, y=2)
        G.add_node(5, x=2, y=4)
        G.add_node(6, x=0, y=4)
        G.add_node(7, x=4, y=4)
        G.add_node(8, x=2, y=6)

        G.add_edge(1, 2, length=60.)
        G.add_edge(2, 3, length=60.)
        G.add_edge(2, 4, length=60.)
        G.add_edge(2, 5, length=60.)
        G.add_edge(5, 6, length=60.)
        G.add_edge(5, 7, length=60.)
        G.add_edge(5, 8, length=60.)

        project_graph_mock.return_value = G
        mdp = mdp6.MDP(G)
        mdp.start = 1
        mdp.goal = 8
        mdp._setup()

        want = [
            mdp6.Intersection(left_node=3, right_node=4, straight_on_node=5, origin_edge=(1, 2, {'length': 60.0})),
            mdp6.Intersection(left_node=6, right_node=7, straight_on_node=8, origin_edge=(2, 5, {'length': 60.0}))
        ]
        self.assertEqual(mdp._get_normal_intersections(), want)

        G.node[3]['x'] = 1
        G.node[3]['y'] = 0

        project_graph_mock.return_value = G
        mdp = mdp6.MDP(G)
        mdp.start = 1
        mdp.goal = 8
        mdp._setup()

        want = [
            mdp6.Intersection(left_node=None, right_node=4, straight_on_node=5, origin_edge=(1, 2, {'length': 60.0})),
            mdp6.Intersection(left_node=6, right_node=7, straight_on_node=8, origin_edge=(2, 5, {'length': 60.0}))
        ]
        self.assertEqual(mdp._get_normal_intersections(), want)

        G.node[4]['x'] = 3
        G.node[4]['y'] = 0

        project_graph_mock.return_value = G
        mdp = mdp6.MDP(G)
        mdp.start = 1
        mdp.goal = 8
        mdp._setup()

        want = [
            mdp6.Intersection(left_node=6, right_node=7, straight_on_node=8, origin_edge=(2, 5, {'length': 60.0}))
        ]
        self.assertEqual(mdp._get_normal_intersections(), want)

    @patch('osmnx.project_graph')
    def test_make_close_intersections_uncertain(self, project_graph_mock):
        G = nx.MultiDiGraph()

        G.add_node(1, x=2, y=0)
        G.add_node(2, x=2, y=2)
        G.add_node(3, x=0, y=2)
        G.add_node(4, x=4, y=2)
        G.add_node(5, x=2, y=4)
        G.add_node(6, x=0, y=4)
        G.add_node(7, x=4, y=4)
        G.add_node(8, x=2, y=6)

        G.add_edge(1, 2, length=60.)
        G.add_edge(2, 3, length=60.)
        G.add_edge(2, 4, length=60.)
        G.add_edge(2, 5, length=60.)
        G.add_edge(5, 6, length=60.)
        G.add_edge(5, 7, length=60.)
        G.add_edge(5, 8, length=60.)

        project_graph_mock.return_value = G
        mdp = mdp6.MDP(G)
        mdp.goal = 3
        mdp._setup()

        want = [
            mdp6.Intersection(left_node=3, right_node=4, straight_on_node=5, origin_edge=(1, 2, {'length': 60.0}))
        ]
        self.assertEqual(mdp.make_close_intersections_uncertain(), want)

        G.node[3]['x'] = 1
        G.node[3]['y'] = 0

        project_graph_mock.return_value = G
        mdp = mdp6.MDP(G)
        mdp.goal = 3
        mdp._setup()

        want = [
            mdp6.Intersection(left_node=None, right_node=4, straight_on_node=5, origin_edge=(1, 2, {'length': 60.0}))
        ]
        self.assertEqual(mdp.make_close_intersections_uncertain(), want)

        G.node[4]['x'] = 3
        G.node[4]['y'] = 0

        project_graph_mock.return_value = G
        mdp = mdp6.MDP(G)
        mdp.goal = 3
        mdp._setup()

        self.assertEqual(mdp.make_close_intersections_uncertain(), [])

        # TODO: More test cases. E.g. if the second intersection is not a
        # 'normal' intersection.

    @patch('osmnx.project_graph')
    def test_drive(self, project_graph_mock):
        G = _setup_mdp()
        project_graph_mock.return_value = G
        mdp = mdp6.MDP(G)
        mdp.start = 1
        mdp.goal = 6
        mdp.A, mdp.C, mdp.P, mdp.S = self._get_initialized_mdp_attrs()
        mdp.C[6, 6] = 0.

        V_want, Q_want = self._get_solved_attrs()

        policy = {1: 0, 2: 1, 3: 0, 4: 0, 5: 0, 6: 1}

        self.assertEqual(mdp.drive(policy, {}), [1, 2, 4, 6])

        diverge_policy = {2: 3}
        self.assertEqual(mdp.drive(policy, diverge_policy), [1, 2, 3, 6])

        # Test whether loops are resolved appropriately
        diverge_policy = {4: 2}
        self.assertEqual(mdp.drive(policy, diverge_policy), [1, 2, 4, 2, 4, 6])


if __name__ == '__main__':
    unittest.main()
