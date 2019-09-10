# cython: language_level=3
from osmnx_mdp.lib cimport get_edge_cost
from osmnx_mdp.algorithms.mdp cimport MDP
from osmnx_mdp.algorithms.dense_hash_map cimport dense_hash_map


cdef class ImprovedDStarLite(osmnx_mdp.algorithms.algorithm.Algorithm):
    def __init__(self, MDP mdp):
        self.mdp = mdp

    cdef setup(self, long start, long goal, unordered_map[string, double] cfg):
        self.cost.set_empty_key((0, 0))
        self.data.set_empty_key(0)
        self.successors.set_empty_key(0)
        self.predecessors.set_empty_key(0)
        self.num_outcomes.set_empty_key(-1)  # lmao

        for node in self.mdp.G.nodes():
            self.num_outcomes[node].set_empty_key((0, 0))

        self.num_outcomes[0].set_empty_key((0, 0))

        for node in self.mdp.G.nodes.data():
            node_id = node[0]

            node_successors = list(self.mdp.G.successors(node_id))
            node_predecessors = list(self.mdp.G.predecessors(node_id))

            self.successors[node_id] = node_successors
            self.predecessors[node_id] = node_predecessors
            self.data[node_id] = (node[1]['lat'], node[1]['lon'])
            for succ in node_successors:
                self.cost[(node_id, succ)] = get_edge_cost(self.mdp.G, node_id, succ)

                for pred in node_predecessors:
                    self.num_outcomes[pred][(node_id, succ)] = len(self.mdp.P[pred][(node_id, succ)])

                # TODO Copied from mdp, duplicate code. Consider lib func.
                has_traffic_signal = self.mdp.G.nodes[succ]['highway'] == 'traffic_signals'
                if has_traffic_signal:
                    # 5 seconds converted to hours.
                    self.cost[(node_id, succ)] += 5 / 3600.

        self.cpp = CPP_Improved_DStar_Lite()
        self.cpp.init(&self.predecessors, &self.successors, &self.cost, &self.data, &self.num_outcomes)
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
