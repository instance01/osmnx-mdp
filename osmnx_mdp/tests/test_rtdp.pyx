# cython: language_level=3
import unittest
from unittest.mock import patch

from osmnx_mdp.tests.test_utils import _setup_mdp
from osmnx_mdp.tests.test_utils import _get_initialized_mdp_attrs

from osmnx_mdp.algorithms.rtdp cimport RTDP
from osmnx_mdp.algorithms.mdp cimport MDP


class TestRTDP(unittest.TestCase):
    pass
    #@patch('numpy.random.choice')
    #def test_run_trials(self, mock_choice):
    #    # Determinize random.choice
    #    mock_choice.side_effect = lambda successors, _, __: [successors[0]]

    #    G = _setup_mdp(remove_dead_ends=True)
    #    mdp = MDP(G)
    #    mdp.start = 1
    #    mdp.goal = 6
    #    mdp.A, mdp.C, mdp.P, mdp.S = _get_initialized_mdp_attrs()

    #    rtdp = RTDP(mdp)
    #    rtdp.setup(start=1, goal=6)
    #    path = rtdp.run_trials(100)
    #    self.assertEqual(path, [1, 2, 3, 6])


if __name__ == '__main__':
    unittest.main()
