import unittest
from unittest.mock import patch
from unittest.mock import PropertyMock

import osmnx as ox

from lib import get_angle
from lib import aerial_dist
from lib import get_edge_cost
from lib import get_time_to_drive
from test_utils import _setup_mdp


class TestLib(unittest.TestCase):
    def test_aerial_dist(self):
        node1 = {'lat': 48.1473983, 'lon': 11.5631933}
        node2 = {'lat': 48.1501208, 'lon': 11.5810993}

        self.assertEqual(aerial_dist(node1, node2), 1.3594507251718166)

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
        self.assertEqual(get_angle((0, 1), (1, 0), (0, 0)), 90.)
        self.assertEqual(get_angle((0, .5), (1, 0), (0, 0)), 90.)
        self.assertEqual(get_angle((.5, .5), (1, 0), (0, 0)), 45.)
        self.assertEqual(get_angle((-.5, -.5), (1, 0), (0, 0)), 225.)
        self.assertEqual(get_angle((-1, 0), (1, 0), (0, 0)), 180.)
        # TODO: More test cases

    def test_get_time_to_drive(self):
        G = _setup_mdp()

        time = get_time_to_drive([1, 2, 4, 6], G)
        self.assertEqual(time, 0.028)

        G.node[1]['lat'] = 48.1473983
        G.node[1]['lon'] = 11.5631933
        G.node[6]['lat'] = 48.1501208
        G.node[6]['lon'] = 11.5810993

        time = get_time_to_drive([1, 6], G)
        self.assertEqual(time, 2.7189014503436333)


if __name__ == '__main__':
    unittest.main()
