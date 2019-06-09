# cython: language_level=3
from osmnx_mdp.algorithms.mdp cimport MDP


# For now unfortunately copy paste from BRTDP wrapper.
# TODO: Does Cython support this kind of polymorphism ? From early tests,
# doesn't seem so.


cdef class BRTDP_REPLAN(osmnx_mdp.algorithms.algorithm.Algorithm):
    def __init__(self, MDP mdp):
        self.mdp = mdp

    cdef setup(self, long start, long goal):
        self.data.set_empty_key(0)
        for node in self.mdp.G.nodes.data():
            node_id = node[0]
            self.data[node_id] = (node[1]['lat'], node[1]['lon'])

        self.cpp = CPP_BRTDP_REPLAN()
        self.cpp.init(
                &self.mdp.S,
                &self.mdp.A,
                &self.mdp.C,
                &self.mdp.P,
                &self.data)
        self.cpp.setup(start, goal)

    cdef run_trials(self):
        self.cpp.run_trials()
        self.vl = self.cpp.vl

    cdef get_path(self, diverge_policy):
        cdef dense_hash_map[long, long] cpp_diverge_policy
        cpp_diverge_policy.set_empty_key(0)

        # Manually convert Python object to Cython object.
        # Unfortunately, there is no other way other than to be explicit.
        for k, v in diverge_policy.items():
            cpp_diverge_policy[k] = v

        # TODO: Bug in Cython? Can't set defaults in CPP_BRTDP_REPLAN in pxd.
        return self.cpp.get_path(cpp_diverge_policy, .02, True)

    cdef solve(self):
        self.run_trials()

    cdef drive(self, policy, diverge_policy):
        return self.get_path(diverge_policy)
