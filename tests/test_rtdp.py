import unittest
from unittest.mock import patch

import mdp6
import rtdp3
from test_utils import _setup_mdp
from test_utils import _get_initialized_mdp_attrs


class TestRTDP(unittest.TestCase):
    @patch('numpy.random.choice')
    def test_run_trials(self, mock_choice):
        # Determinize random.choice
        mock_choice.side_effect = lambda successors, _, __: [successors[0]]

        G = _setup_mdp(remove_dead_ends=True)
        mdp = mdp6.MDP(G)
        mdp.start = 1
        mdp.goal = 6
        mdp.A, mdp.C, mdp.P, mdp.S = _get_initialized_mdp_attrs()

        rtdp = rtdp3.RTDP(mdp)
        rtdp.setup(start=1, goal=6)
        path = rtdp.run_trials(100)
        self.assertEqual(path, [1, 2, 3, 6])


if __name__ == '__main__':
    unittest.main()
