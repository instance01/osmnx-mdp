# cython: language_level=3
from osmnx_mdp.algorithms.mdp cimport MDP


# TODO: extend Algorithm
cdef class BRTDP:
    def __init__(self, MDP mdp):
        self.mdp = mdp

    cdef setup(self, long start, long goal):
        self.data.set_empty_key(0)
        for node in self.mdp.G.nodes.data():
            node_id = node[0]
            self.data[node_id] = (node[1]['lat'], node[1]['lon'])

        self.S = self.mdp.S

        self.A.set_empty_key(0)
        for k, v in self.mdp.A.items():
            self.A[k] = v

        self.C.set_empty_key((0, 0))
        for k, v in self.mdp.C.items():
            self.C[k] = v

        self.P.set_empty_key(0)
        cdef dense_hash_map[pair[long, long], vector[pair[long, double]], pair_hash] curr
        curr.set_empty_key((0, 0))
        for k, v in self.mdp.P.items():
            for k_, v_ in v.items():
                curr[k_] = v_
            self.P[k] = curr
            curr.clear()

        self.cpp = CPP_BRTDP()
        self.cpp.init(&self.S, &self.A, &self.C, &self.P, &self.data)
        self.cpp.setup(start, goal)

    cdef run_trials(self):
        self.cpp.run_trials()
        self.vl = self.cpp.vl

    cdef get_path(self):
        return self.cpp.get_path()
