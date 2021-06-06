import networkx as nx
import pickle
import plotly.plotly as py
import random
import matplotlib.pyplot as plt
from plotly.graph_objs import *
from plotly.offline import init_notebook_mode, plot, iplot

init_notebook_mode(connected=True)
store_names= {0:'A',1:'B',2:'C',3:'D',4:'F',5:'G',6:'H',7:'I',8:'J',9:'K',
              10:'L',11:'M',12:'N',13:'O',14:'P',15:'Q'}

map_10_dict = {
    0: {'pos': (0.7798606835438107, 0.6922727646627362), 'connections': [7, 6, 10]},
    1: {'pos': (0.7647837074641568, 0.3252670836724646), 'connections': [4, 5, 2]},
    2: {'pos': (0.7155217893995438, 0.20026498027300055), 'connections': [7, 3, 1]},
    3: {'pos': (0.7076566826610747, 0.3278339270610988), 'connections': [8, 9, 10, 12]},
    4: {'pos': (0.8325506249953353, 0.02310946309985762), 'connections': [1, 2, 3]},
    5: {'pos': (0.49016747075266875, 0.5464878695400415), 'connections': [7, 0, 3]},
    6: {'pos': (0.8820353070895344, 0.6791919587749445), 'connections': [0]},
    7: {'pos': (0.46247219371675075, 0.6258061621642713), 'connections': [0, 5]},
    8: {'pos': (0.11622158839385677, 0.11236327488812581), 'connections': [1,9]},
    9: {'pos': (0.1285377678230034, 0.3285840695698353), 'connections': [2,8]},
    10: {'pos': (0.1285377678230034, 0.3285840695698353), 'connections': [4,9]},
    11: {'pos': (0.11622158839385677, 0.11236327488812581), 'connections': [6,10]},
    12: {'pos': (0.7076566826610747, 0.3278339270610988), 'connections': [5, 4,12]},
    13: {'pos': (0.7076566826610747, 0.3278339270610988), 'connections': [11,13]},
    14: {'pos': (0.7076566826610747, 0.3278339270610988), 'connections': [9,12,14]},
    15: {'pos': (0.7076566826610747, 0.3278339270610988), 'connections': [8,11,14,15]},
}




class Map:
    def __init__(self, G):
        self._graph = G
        self.intersections = nx.get_node_attributes(G, "pos")
        self.roads = [list(G[node]) for node in G.nodes()]

    def save(self, filename):
        with open(filename, 'wb') as f:
            pickle.dump(self._graph, f)


def load_map_graph(map_dict):
    G = nx.Graph()
    for node in map_dict.keys():
        G.add_node(node, pos=map_dict[node]['pos'])
    for node in map_dict.keys():
        for con_node in map_dict[node]['connections']:
            G.add_edge(node, con_node)
    return G


def load_map_10():
    G = load_map_graph(map_10_dict)
    return Map(G)





def show_map(M,path=None,dist=None):
    G = M._graph

    pos = nx.spring_layout(G)
    node_size = []
    #nx.draw_networkx(G)
    if path != None:
        edges = G.edges()
        route_edges = [(path[n], path[n + 1]) for n in range(len(path) - 1)]
        route_edges_rev = [(path[n + 1],path[n]) for n in range(len(path) - 1)]
        color = []

        for ed in edges:
            if ed in route_edges or ed in route_edges_rev:
                color.append('g')
            else:
                color.append('r')
        node_color =[]
        for g in G.nodes():
            node_size.append(1100)
            if g in path:
                node_color.append('y')
            else:
                node_color.append('r')
        distances = {}
        i = 0
        for edge in route_edges:
            distances.update({edge:dist[i]})
            i+=1



        nx.draw(G, pos, edges=edges, edge_color=color,node_color=node_color,edge_labels = distances)
        nx.draw_networkx_labels(G, pos, store_names)
        nx.draw_networkx_edge_labels(G, pos, edge_labels= distances)

    plt.show()
