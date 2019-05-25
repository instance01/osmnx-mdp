# cython: language_level=3

cdef get_edge_cost(G, node_from, node_to)
cdef get_angle(p1, p2, origin)
cdef aerial_dist(node1, node2, R=*)
cdef get_time_to_drive(col_nodes, G)
cdef get_node_properties(G, path, values=*, extra=*)
cdef draw_value_graph(G, path, values=*, extra=*, annotate=*)
cdef remove_zero_cost_loops(G)
cdef remove_dead_ends(G, goal)

