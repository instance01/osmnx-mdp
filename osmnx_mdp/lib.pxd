# cython: language_level=3


cdef extern from "cpp_lib.hpp":
    cdef double get_angle(
        double p1_x,
        double p1_y,
        double p2_x,
        double p2_y,
        double origin_x,
        double origin_y)
    cdef double cpp_aerial_dist "aerial_dist"(
        double lat1,
        double lon1,
        double lat2,
        double lon2,
        double R)


cdef get_edge_cost(G, node_from, node_to)
cdef aerial_dist(node1, node2, R=*)
cdef get_time_to_drive(col_nodes, G)
cdef get_node_properties(G, path, values=*, extra=*)
cdef draw_value_graph(G, path, values=*, extra=*, annotate=*)
cdef remove_zero_cost_loops(G)
cdef remove_dead_ends(G, goal)
cdef filter_connected_points(G)
