# cython: language_level=3
# cython: profile=True
from libcpp.pair cimport pair
from libcpp.vector cimport vector

from osmnx_mdp.lib cimport get_edge_cost


cdef class MDP(osmnx_mdp.algorithms.algorithm.Algorithm):
    def __init__(self, G):
        self.G = G
        self.cpp = CPP_MDP()

    cdef _setup_cpp(self):
        self.successors.set_empty_key(0)
        self.node_data.set_empty_key(0)
        self.edge_data.set_empty_key((0, 0))
        self.A.set_empty_key(0)
        self.P.set_empty_key(0)
        self.C.set_empty_key((0, 0))

        for node in self.G.nodes.data():
            node_id = node[0]
            successors = list(self.G.successors(node_id))

            self.successors[node_id] = successors

            self.node_data[node_id] = (node[1]['x'], node[1]['y'])

            self.S.push_back(node_id)
            self.A[node_id] = [(node_id, succ) for succ in successors]

            if self.P.find(node_id) == self.P.end():
                self.P[node_id].set_empty_key((0, 0))

            for succ in successors:
                action = (node_id, succ)
                # For now we end up in correct state 100% of the time.
                self.P[node_id][action].push_back((succ, 1.0))
                self.C[action] = get_edge_cost(self.G, node_id, succ)

                self.edge_data[(node_id, succ)] = self.G[node_id][succ][0]['length']

    cdef setup(self, long start, long goal):
        self._setup_cpp()
        self.cpp.init(
                &self.S,
                &self.A,
                &self.C,
                &self.P,
                &self.edge_data,
                &self.node_data,
                &self.successors)
        self.cpp.setup(start, goal)
        self.uncertain_nodes = set()
        for x in self.cpp.uncertain_nodes:
            self.uncertain_nodes.add(x)

    cdef solve_value_iteration(
            self,
            float gamma=1.,
            int max_iter=50000,
            double eps=1e-20,
            bint verbose=True):
        """Solve the MDP with value iteration.
        """
        self.cpp.solve(max_iter, eps)

        # Direct assignment is not possible since Cython doesnt know how to
        # convert a dense_hash_map to a Python object.
        # But iterators work.
        V = {}
        for k, v in self.cpp.V:
            V[k] = v

        return V

    cdef get_policy(self, V):
        self.cpp.get_policy()

        # Manually convert Cython object into a Python object.
        policy = {}
        for k, v in self.cpp.policy:
            policy[k] = v
        return policy

    cdef drive(self, policy, diverge_policy):
        # Make sure we don't loop indefinitely due to diverge policy
        cdef dense_hash_map[long, long] cpp_diverge_policy
        cpp_diverge_policy.set_empty_key(0)

        # Manually convert Python object into a Cython object.
        for k, v in diverge_policy.items():
            cpp_diverge_policy[k] = v

        return self.cpp.drive(cpp_diverge_policy)

    cdef solve(self):
        V = self.solve_value_iteration()
        return self.get_policy(V)
