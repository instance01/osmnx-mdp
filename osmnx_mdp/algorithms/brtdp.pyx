# cython: language_level=3
from osmnx_mdp.algorithms.mdp cimport MDP


cdef class BRTDP(osmnx_mdp.algorithms.algorithm.Algorithm):
    def __init__(self, MDP mdp):
        self.mdp = mdp

    cdef setup(self, long start, long goal, unordered_map[string, double] cfg):
        self.data.set_empty_key(0)
        self.predecessors.set_empty_key(0)

        for node in self.mdp.G.nodes.data():
            node_id = node[0]
            self.data[node_id] = (node[1]['x'], node[1]['y'])
            self.predecessors[node_id] = list(self.mdp.G.predecessors(node_id))

        self.cpp = CPP_BRTDP()
        self.cpp.init(
                &self.mdp.S,
                &self.mdp.A,
                &self.mdp.C,
                &self.mdp.P,
                &self.predecessors,
                &self.data)
        self.cpp.setup(start, goal, cfg)

    cdef run_trials(self):
        self.iterations = self.cpp.run_trials()
        self.vl = self.cpp.vl

    cdef get_path(self, diverge_policy):
        cdef dense_hash_map[long, long] cpp_diverge_policy
        cpp_diverge_policy.set_empty_key(0)

        # Manually convert Python object to Cython object.
        # Unfortunately, there is no other way other than to be explicit.
        for k, v in diverge_policy.items():
            cpp_diverge_policy[k] = v

        return self.cpp.get_path(cpp_diverge_policy)

    cdef solve(self):
        self.run_trials()

    cdef drive(self, policy, diverge_policy):
        return self.get_path(diverge_policy)
