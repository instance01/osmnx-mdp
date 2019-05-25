# cython: language_level=3
import unittest
from unittest.mock import patch

from osmnx_mdp.tests.test_utils import _setup_mdp

from osmnx_mdp.simulation import run_simulation
from osmnx_mdp.simulation import run_simulations
from osmnx_mdp.simulation import generate_diverge_policy

from osmnx_mdp.algorithms.mdp cimport MDP


class TestSimulation(unittest.TestCase):
    def test_load_maps(self):
        # TODO
        pass

    @patch('osmnx.project_graph')
    def test_run_simulation(self, mock_obj):
        G = _setup_mdp(remove_dead_ends=True)
        mock_obj.return_value = G
        mdp = MDP(G)
        mdp.start = 1
        mdp.goal = 6

        # When we get to state 2, best path would be to go state 4.
        # We diverge to 3.
        diverge_policy = {2: 3}

        path = run_simulation(mdp, None, 1, 6, diverge_policy)
        self.assertEqual(path, [1, 2, 3, 6])

    @patch('numpy.random.choice')
    @patch('random.random')
    def test_generate_diverge_policy(self, mock_random, mock_choice):
        # Determinize random
        random_seq = iter([.1, .9, .9, .1, .1, .9])
        mock_random.side_effect = lambda: next(random_seq)
        mock_choice.side_effect = lambda successors, _: [successors[0]]

        G = _setup_mdp()
        diverge_policy = generate_diverge_policy(G)
        self.assertEqual(diverge_policy, {1: 2, 4: 6})

    @patch('numpy.random.choice')
    @patch('random.random')
    # Don't import mdp.MDP, but simulation.MDP due to the from import in
    # simulation.py
    #@patch('simulation.MDP.setup', autospec=True)
    @patch('osmnx_mdp.simulation.load_maps', spec=lambda: None)
    @patch('osmnx_mdp.simulation.remove_dead_ends', spec=lambda: None)
    @patch('osmnx.project_graph')
    def test_run_simulations(
            self,
            mock_project_graph,
            mock_remove_dead_ends,
            mock_load_maps,
            #mock_MDP_setup,
            mock_random,
            mock_choice):
        # It's important that the diverge policy stays the same for each
        # simulation.

        # Determinize random
        random_seq = iter([.9, .1, .9, .9, .1, .1])
        mock_random.side_effect = lambda: next(random_seq)
        mock_choice.side_effect = lambda successors, _, __=False: [successors[0]]

        G = _setup_mdp(remove_dead_ends=True)
        mock_project_graph.return_value = G

        results = []
        orig = run_simulation

        def run_with_results(
                algo, map_id, start, goal, diverge_policy, call_again=True):
            if call_again:
                res = orig(algo, map_id, start, goal, diverge_policy, False)
                results.append(res)
                return res

        with patch('osmnx_mdp.simulation.run_simulation') as mock_run_simulation:
            mock_run_simulation.side_effect = run_with_results

            MAPS = {
                'test': {
                    'map': G
                }
            }

            LOCATIONS = {
                'test': [
                    {
                        'start': 1,
                        'goal': 6
                    }
                ]
            }

            with patch('osmnx_mdp.simulation.MAPS', MAPS), patch('osmnx_mdp.simulation.LOCATIONS', LOCATIONS):
                run_simulations()

            self.assertEqual(results, [[1, 2, 3, 6], [1, 2, 3, 6], {1, 2, 3, 6}])
