import unittest

import mdp6
import rtdp3
from test_utils import _setup_mdp


class TestRTDP(unittest.TestCase):
    def test_run_trials(self):
        G = _setup_mdp()
        mdp = mdp6.MDP(G)
        mdp.start = 1
        mdp.goal = 6
        mdp.remove_zero_cost_loops()
        mdp._setup()
        mdp.make_goal_self_absorbing()

        rtdp = rtdp3.RTDP(mdp)
        rtdp.start = 1
        rtdp.goal = 6
        rtdp.setup()
        path = rtdp.run_trials(10)
        self.assertEqual(path, [1, 2, 4, 6])


if __name__ == '__main__':
    unittest.main()
