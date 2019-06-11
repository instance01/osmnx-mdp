# cython: language_level=3
import unittest
from unittest.mock import patch
from unittest.mock import PropertyMock

import networkx as nx

from osmnx_mdp.lib cimport get_angle
from osmnx_mdp.lib cimport aerial_dist
from osmnx_mdp.lib cimport get_edge_cost
from osmnx_mdp.lib cimport get_time_to_drive
from osmnx_mdp.lib cimport remove_zero_cost_loops
from osmnx_mdp.lib cimport remove_dead_ends
from osmnx_mdp.tests.test_utils import _setup_mdp


class TestLib(unittest.TestCase):
    def test_aerial_dist(self):
        node1 = {'lat': 48.1473983, 'lon': 11.5631933}
        node2 = {'lat': 48.1501208, 'lon': 11.5810993}

        self.assertEqual(aerial_dist(node1, node2), 1.359450725171816)

        # Since we need an admissible heuristic, simply make sure not to
        # over estimate the distance.
        # Thus we use the lower bound, which is polar radius.
        # To test whether this makes a huge difference, the aerial distance
        # between two points in Maxvorstadt were calculated with
        # (1) polar radius
        # (2) equatorial radius
        # as parameter R (radius of earth).
        # We see that there is only a difference of roughly 4.6 metres.
        # Thus we conclude that using polar radius as parameter R is good
        # enough.

        # Radius values as per IAU:
        # Mamajek, E. E., et al. "IAU 2015 resolution B3 on recommended
        # nominal conversion constants for selected solar and planetary
        # properties." arXiv preprint arXiv:1510.07674 (2015).
        polar_radius = 6356.8
        equatorial_radius = 6378.1

        difference = 0.00455516933774236  # km
        lowest_estimate = aerial_dist(node1, node2, polar_radius)
        highest_estimate = aerial_dist(node1, node2, equatorial_radius)
        self.assertEqual(highest_estimate - lowest_estimate, difference)

    @patch('networkx.MultiDiGraph.edges', new_callable=PropertyMock)
    def test_get_edge_cost(self, mock_obj):
        mock_obj.return_value = {
            (1, 2, 0): {'maxspeed': [1], 'length': 30.0}
        }
        G = _setup_mdp()
        self.assertEqual(get_edge_cost(G, 1, 2), 0.001)

    def test_get_angle(self):
        self.assertEqual(get_angle(0, 1, 1, 0, 0, 0), 90.)
        self.assertEqual(get_angle(0, .5, 1, 0, 0, 0), 90.)
        self.assertEqual(get_angle(.5, .5, 1, 0, 0, 0), 45.)
        self.assertEqual(get_angle(-.5, -.5, 1, 0, 0, 0), 225.)
        self.assertEqual(get_angle(-1, 0, 1, 0, 0, 0), 180.)
        # TODO: More test cases

    def test_get_time_to_drive(self):
        G = _setup_mdp()

        time = get_time_to_drive([1, 2, 4, 6], G)
        self.assertEqual(time, 0.038)

        G.node[1]['lat'] = 48.1473983
        G.node[1]['lon'] = 11.5631933
        G.node[6]['lat'] = 48.1501208
        G.node[6]['lon'] = 11.5810993

        time = get_time_to_drive([1, 6], G)
        self.assertEqual(time, 2.7189014503436333)

    def test_remove_zero_cost_loops(self):
        G = nx.MultiDiGraph()

        G.add_node(1, x=0, y=0)
        G.add_node(2, x=2, y=5)
        G.add_node(3, x=2, y=0)

        G.add_edge(2, 3, length=6.)
        G.add_edge(3, 3, length=6.)
        G.add_edge(2, 1, length=4.)

        remove_zero_cost_loops(G)

        self.assertEqual(list(G.edges()), [(2, 3), (2, 1)])

    def test_remove_dead_ends(self):
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

        remove_dead_ends(G, 8)

        self.assertEqual(list(G.nodes()), [1, 2, 3, 4, 8])

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

        remove_dead_ends(G, 3)

        self.assertEqual(list(G.nodes()), [1, 2, 3, 4])

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

        remove_dead_ends(G, 3)

        self.assertEqual(list(G.nodes()), [1, 2, 3, 4])


if __name__ == '__main__':
    unittest.main()
