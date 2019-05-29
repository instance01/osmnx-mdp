Solving routing with uncertainty using Markov decision processes.


A driver follows along a route, but diverges from it.
What do?
This is uncertainty - we don't know when or if this will happen.


Algorithms implemented right now:
* Value iteration
* RTDP
* LPA*
* BRTDP
* D* Lite

Algorithms planned:
* ID* Lite ?
* D* Lite improved with HOP ?

Miscellaneous methods:
* Angles using arctan2
* Haversine

Technologies:
* osmnx (OpenStreetMap + networkx)


Development:
pip3 install -e .
Install sparsehash package using your systems package manager
