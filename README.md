Solving routing with uncertainty using Markov decision processes.

A driver follows along a route, but diverges from it. What do? This is
uncertainty - we don't know when or if this will happen. The repo contains
project files to compare different methods of shortest path finding in an
uncertain environment on different maps such as Bavaria. There is no dynamic
costs - the only use case is a driver making a wrong turn. This repo serves as
a good real world application of the algorithms below.

Algorithms implemented right now:
* Value iteration
* BRTDP with DS-MPI
* D* Lite

Dependencies:
* osmnx (OpenStreetMap + networkx)
* Google's dense hashmap (sparsehash package)

Running/Development:
```
sudo docker build -t osmnx-mdp -f Dockerfile .
sudo docker run -v $(pwd):/app -it osmnx-mdp bash
```

There's a few custom commands in the image (contrib/bashrc), most notably setup and run2.

To run the current simulation:
```
cd osmnx_mdp
setup
run2 runner
```

![States visited by BRTDP](https://raw.githubusercontent.com/instance01/osmnx-mdp/master/.github/brtdp_band2.png)

Goodies for learners and code readers:
* np.rec.array and numpy indexing: visualize.py, extract_for_pgfplots.py
* Adding a custom data structure to Cython: dense_hash_map.pxd
* Cython inheritance: algorithm.pxd/pyx
* DBL_EPSILON: cpp_brtdp.cpp
* Very simple min-priority queue with decrease-key operation (no extra class): cpp_queue_util.hpp
* Design pattern 'Strategy': simulation.pyx, run_simulation function
* Multiprocessing pool example: simulation.pyx
* Simple example on how to do the t-test for the means of two samples, using ttest_ind: stat_test/
