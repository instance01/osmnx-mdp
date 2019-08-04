# cython: language_level=3
import math

import numpy as np
import osmnx as ox
import networkx as nx
from networkx.algorithms.components.connected import connected_components


# TODO Move everything here into cpp_lib


cdef get_edge_cost(G, node_from, node_to):
    """Edge cost is the amount of time (in hours) an edge takes to travel, i.e.
    length divided by maxspeed.
    """
    # The 0 below is the networkx key, when you have multiple edges
    # in parallel, to distinguish them.
    # In osmnx context this should never happen, but this should be tested.
    edge_data = G.edges[node_from, node_to, 0]

    # https://wiki.openstreetmap.org/wiki/Key:maxspeed
    # TODO: Magic numbers
    maxspeed = edge_data.get('maxspeed', 30)
    if maxspeed == 'none':
        maxspeed = 120  # TODO: Reconsider this
    elif maxspeed == 'signals':
        maxspeed = 50
    elif maxspeed == 'walk':
        maxspeed = 30
    elif isinstance(maxspeed, list):
        maxspeed = 30
    else:
        try:
            maxspeed = int(maxspeed)
        except ValueError:
            maxspeed = 30

    length = edge_data['length'] / 1000.  # km
    return length / maxspeed


cdef aerial_dist(node1, node2, R=6356.8):
    return cpp_aerial_dist(
        <double>node1['lat'],
        <double>node1['lon'],
        <double>node2['lat'],
        <double>node2['lon'],
        <double>R
    )


cdef get_time_to_drive(col_nodes, G):
    minutes_multiplier = 60.
    time_to_drive = 0  # Minutes

    # TODO: For later: Expensive loop.
    for (node_from, node_to) in zip(col_nodes, col_nodes[1:]):
        if node_to in G[node_from]:
            cost = get_edge_cost(G, node_from, node_to)
            time_to_drive += cost * minutes_multiplier
        else:
            node_from = G.nodes[node_from]
            node_to = G.nodes[node_to]
            dist = aerial_dist(node_from, node_to)
            maxspeed = 30.
            time_to_drive += dist / maxspeed * minutes_multiplier

    return time_to_drive


cdef get_node_properties(G, path, values=None, extra=[]):
    # TODO: Rename function
    """G shall be projected.
    values is a dict of {node: cost, ..}.
    """
    if values is not None:
        p95_val = np.percentile(list(values.values()), 95)
        min_val = min(values.values())
        diff_val = p95_val - min_val

    HEX_FORMAT = '#ff%s00'

    # TODO Absolutely lmao code quality

    node_sizes = []
    node_colors = []
    for node in G.nodes():
        if node in path:
            # TODO Ugly
            if node in extra:
                node_colors.append('m')
            else:
                node_colors.append('g')
            node_sizes.append(30)
        elif node in extra:
            node_colors.append('y')
            node_sizes.append(20)
        else:
            node_sizes.append(8)
            if values is None:
                node_colors.append('#555555')
                continue
            val = values.get(node, None)
            if not val:
                node_colors.append('#555555')
                continue
            val = max(0., val)
            val = min(p95_val, val)
            val = hex(int(254 - (val - min_val) / diff_val * 254))[2:4].zfill(2)
            node_colors.append(HEX_FORMAT % val)

    return node_colors, node_sizes


cdef draw_value_graph(G, path, values=None, extra=[], annotate=False):
    """G shall be projected.
    values is a dict of {node: cost, ..}.
    """
    node_colors, node_sizes = get_node_properties(G, path, values, extra)

    ox.plot_graph(
            G,
            node_size=node_sizes,
            node_color=node_colors,
            node_zorder=2,
            annotate=annotate)


cdef remove_zero_cost_loops(G):
    """Apparently osmnx gives us looping edges (that go back to itself) on
    the border of the map. The problem is that they're zero-cost loops.
    Let's just remove them.
    """
    loops = [edge for edge in G.edges() if edge[0] == edge[1]]
    for loop in loops:
        G.remove_edge(*loop)


cdef remove_dead_ends(G, goal):
    """Since the map (in current tests Maxvorstadt) is 'cut out' of Munich,
    some paths leave the map without coming back. This means if a driver
    were to follow this path, he would not be able to come back, so this
    is a dead end.
    Value iteration does not converge with dead ends.
    Since we know where our dead ends lie, we can simply filter them out
    and don't have to resort to other models such as fSSPUDE etc. TODO
    Paper: Kolobov, Stochastic Shortest Path MDPs with Dead Ends
    """
    while True:
        # TODO: This possibly doesn't have to be a set.
        todel = set()

        candidates = []
        for node in G.nodes():
            if node == goal:
                continue

            predecessors = list(G.predecessors(node))
            successors = list(G.successors(node))

            has_loop = predecessors == successors and len(predecessors) == 1
            if not successors or has_loop:
                candidates.append(node)

        for candidate in candidates:
            open_ = set(G.predecessors(candidate))
            open_.add(candidate)

            while open_:
                node = open_.pop()

                if node in todel:
                    continue

                successors = list(G.successors(node))
                if len(successors) <= 1:
                    open_.update(G.predecessors(node))
                    todel.add(node)

        if not todel:
            break

        for node in todel:
            G.remove_node(node)


cdef filter_connected_points(G):
    # TODO: This is not used right now.
    return []

    G_candidates = nx.Graph()

    for node in G.nodes.data():
        node_id = node[0]

        for succ in G.successors(node_id):
            length = G[node_id][succ][0]['length']
            if length < 20:
                G_candidates.add_edge(node_id, succ)

    candidates = list(connected_components(G_candidates))

    # TODO REMOVE
    pts = []

    for candidate in candidates:
        pred_metadata = {}
        succ_metadata = {}
        predecessors = []
        successors = []

        for node in candidate:
            for pred in G.predecessors(node):
                if pred not in candidate:
                    predecessors.append(pred)
                    pred_metadata[pred] = G[pred][node][0]

            for succ in G.successors(node):
                if succ not in candidate:
                    successors.append(succ)
                    succ_metadata[succ] = G[node][succ][0]

        xs = [G.nodes[node]['x'] for node in candidate]
        ys = [G.nodes[node]['y'] for node in candidate]

        center = {
            'x': np.average(xs),
            'y': np.average(ys),
            'lat': np.average(xs),
            'lon': np.average(ys)
        }

        # TODO: This line is intentionally buggy. At least for now.
        node_id = next(iter(candidate))

        for node in candidate:
            G.remove_node(node)

        G.add_node(node_id, **center)
        pts.append(node_id)

        for pred in predecessors:
            G.add_edge(pred, node_id, **pred_metadata[pred])

        for succ in successors:
            G.add_edge(node_id, succ, **succ_metadata[succ])

    print('Done filtering')
    return pts
