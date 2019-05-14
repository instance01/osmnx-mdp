import pickle
import osmnx as ox


place = 'Maxvorstadt, Munich, Germany'

ox.config(use_cache=True)
G = ox.graph_from_place(place, network_type='drive', simplify=True)
G = ox.add_edge_lengths(G)

with open('maxvorstadt2.pickle', 'wb+') as f:
    pickle.dump(G, f)
