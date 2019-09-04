# cython: language_level=3
from osmnx_mdp.lib cimport get_edge_cost

from osmnx_mdp.algorithms.dense_hash_map cimport dense_hash_map


# TODO: unittests


cdef class DStarLite(osmnx_mdp.algorithms.algorithm.Algorithm):
    def __init__(self, G):
        self.G = G

    cdef setup(self, long start, long goal, unordered_map[string, double] cfg):
        self.cost.set_empty_key((0, 0))
        self.data.set_empty_key(0)
        self.successors.set_empty_key(0)
        self.predecessors.set_empty_key(0)

        for node in self.G.nodes.data():
            node_id = node[0]

            node_successors = list(self.G.successors(node_id))
            node_predecessors = list(self.G.predecessors(node_id))

            self.successors[node_id] = node_successors
            self.predecessors[node_id] = node_predecessors
            self.data[node_id] = (node[1]['lat'], node[1]['lon'])
            for succ in node_successors:
                self.cost[(node_id, succ)] = get_edge_cost(self.G, node_id, succ)

                # TODO Copied from mdp, duplicate code. Consider lib func.
                has_traffic_signal = self.G.nodes[succ]['highway'] == 'traffic_signals'
                if has_traffic_signal:
                    # 5 seconds converted to hours.
                    self.cost[(node_id, succ)] += 5 / 3600.

        self.cpp = CPP_DStar_Lite()
        self.cpp.init(&self.predecessors, &self.successors, &self.cost, &self.data)
        self.cpp.setup(start, goal, cfg)

    cdef compute_shortest_path(self):
        self.iterations = self.cpp.compute_shortest_path()

    cdef solve(self):
        self.compute_shortest_path()

    cdef drive(self, policy, diverge_policy):
        cdef dense_hash_map[long, long] diverge_policy_cpp
        diverge_policy_cpp.set_empty_key(0)

        # TODO: Copied from BRTDP
        # Manually convert Python object to Cython object.
        # Unfortunately, there is no other way other than to be explicit.
        for k, v in diverge_policy.items():
            diverge_policy_cpp[k] = v

        cdef vector[long] path
        self.cpp.drive(path, diverge_policy_cpp)
        return path
