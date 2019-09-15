This is a list of published simulations with their configurations.
Simulation pickles: tar -xJf simulationpickles.tar.xz
Corresponding command outputs: unzip outputs.zip

simulationR1: default
simulationR2: MDP
simulationR3: edge_uncertainty=.2 (ff2.txt)
simulationR4: edge_uncertainty=.3 random_chance=.3 (tt.txt)
simulationR5: max_length=100 (decrease n of nodes uncertain 17% -> 12% BY) (tt2.txt)
simulationR6: max_angle=20, max_length=50 (decrease n of nodes uncertain 17% -> 4% BY (vs R4)) (tt3.txt)
simulationR7: max_angle=35, max_length=250, edge_uncertainty=.3 (tt4.txt)
simulationR8: max_angle=35, max_length=250, edge_uncertainty=.3, diverge_policy=model (tt5.txt)
simulationR9: max_angle=35, max_length=250, edge_uncertainty=.3, random_chance=.3 (tt6.txt)
simulationR10: dijkstra (ss.txt)
simulationR11: random_chance=.05, dijkstra (ss2.txt)
simulationR12: diverge_policy=model, dijkstra (ss3.txt)
simulationR13: max_length=50, diverge_policy=model, dijkstra
simulationR14: diverge_policy=model, BRTDP dijkstra (ss5.txt)
simulationR15: (all maps)_ random, .2 (uu1.txt)
simulationR16: (all maps)_ random, .2, low uncertainty (uu2.txt)
simulationR17: (all maps)_ model, low uncertainty (uu3.txt)
simulationR18: (all maps)_ model, high uncertainty (default) (uu4.txt)
simulationR19: (all maps)_ random, .05, high uncertainty (default) (uu5.txt)
simulationR20: (all maps)_ random, .05, low uncertainty (uu6.txt)
simulationR21: (all maps)_ random, .1, medium uncertainty (angle=20, length=100), 100 ITERS

simulationS1: random, .2, 30, 200 (high uncertainty), D*IMPROV
simulationS2: random, .05, 30, 200 (high uncertainty), D*IMPROV
simulationS3: model, 30, 200 (high uncertainty), D*IMPROV
simulationS4: model, 20, 50 (low uncertainty), D*IMPROV
simulationS5: random, .2, 20, 50 (low uncertainty), D*IMPROV
simulationS6: model, 30, 200 (high uncertainty), Normal D* <<<< like R18

simulationT1: model, 30, 250 (ww1)
simulationT2: random, .2, 30, 250 (ww2)
simulationT3: random, .05, 30, 250 (ww3)
simulationT4: random, .2, 20, 50 (ww4)
simulationT5: random, .05, 20, 50 (ww5)
simulationT6: random, .05, 30, 200 (ww6)
simulationT7: model, 20, 50 (ww7)
