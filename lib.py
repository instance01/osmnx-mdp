import math

import numpy as np
import osmnx as ox


def get_edge_cost(G, node_from, node_to):
    # TODO: Explain that 0.. It's networkx key, when you have multiple edges
    # in parallel, to distinguish them.
    # In osmnx context this should never happen, but this should be tested.
    edge_data = G.edges[node_from, node_to, 0]

    if isinstance(edge_data.get('maxspeed'), list):
        maxspeed = 30
    else:
        maxspeed = int(edge_data.get('maxspeed', 30))

    length = edge_data['length'] / 1000.  # km
    return length / maxspeed


def get_angle(p1, p2, origin):
    p1 = np.array(p1) - origin
    p2 = np.array(p2) - origin

    # arctan2 gives us the angle between the ray from the origin to the point
    # and the x axis.
    # Thus, to get the angle between two points, simply get the difference
    # between their two angles to the x axis.
    angle = np.arctan2(*p2) - np.arctan2(*p1)
    return np.degrees(angle)


def aerial_dist(node1, node2, R=6356.8):
    # TODO: Use osmnx
    # great_circle_vec(lat1, lng1, lat2, lng2, earth_radius=6371009)
    """Haversine
    R shall be radius of earth in km.

    Returns aerial distance in km.

    Source:
    R. W. Sinnott, "Virtues of the Haversine", Sky and Telescope 68 (2), p.159 (1984).
    """
    lon1 = math.radians(node1['lon'])
    lon2 = math.radians(node2['lon'])
    lat1 = math.radians(node1['lat'])
    lat2 = math.radians(node2['lat'])
    d = math.sin((lat2 - lat1) / 2) ** 2 + \
        math.cos(lat1) * \
        math.cos(lat2) * \
        math.sin((lon2 - lon1) / 2) ** 2
    return R * 2 * math.asin(d ** .5)


def get_time_to_drive(col_nodes, G):
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


def get_node_properties(G, path, values=None, extra=[]):
    # TODO: Rename function
    """G shall be projected.
    values is a dict of {node: cost, ..}.
    """
    if values is not None:
        p95_val = np.percentile(list(values.values()), 95)
        min_val = min(values.values())
        diff_val = p95_val - min_val

    HEX_FORMAT = '#ff%s00'

    node_colors = []
    for node in G.nodes():
        if node in path:
            node_colors.append('g')
        elif node in extra:
            node_colors.append('m')
        else:
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

    node_sizes = [50 if node in path or node in extra else 8 for node in G.nodes()]

    return node_colors, node_sizes


def draw_value_graph(G, path, values=None, extra=[], annotate=False):
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
