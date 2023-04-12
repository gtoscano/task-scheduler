#!/usr/bin/env python
# -*- coding: utf-8 -*-

import networkx as nx
import matplotlib.pyplot as plt
import json


def read_file(filename):
    nodes = []
    with open(filename, 'r') as f:
        for line in f:
            u, v, w = map(int, line.strip().split())
            nodes.append((u, v, w))
    return nodes
def read_tree_file(filename):
    nodes = []
    data = []
    with open(filename, 'r') as f:
        data = json.load(f)
        print(data)
        #data = json.loads(f)

    if 'tasks' in data.keys():
        tasks = {int(k): v for k, v in data['tasks'].items()}

    if 'dependencies' in data.keys():
        dependencies = data['dependencies']

    for item in dependencies:
        print(item)
        print(tasks[item[0]])
        nodes.append(item+[tasks[item[0]]])

    return nodes


def plot_graph(nodes):
    # Create a tree graph with capacities
    G.add_weighted_edges_from(nodes);
    #    [(1,2,5), (1,3,8), (2,4,3), (2,5,4), (3,6,6), (3,7,2), (4,8,1), (4,9,7), (5,10,9), (6,11,2)])
    
    # Add labels to the nodes
    node_labels = {1: 'Root', 2: 'A', 3: 'B', 4: 'C', 5: 'D', 6: 'E', 7: 'F', 8: 'G', 9: 'H', 10: 'I', 11: 'J'}
    
    # Plot the tree diagram with labels and capacities
    pos = nx.nx_agraph.graphviz_layout(G, prog='dot')
    #nx.draw(G, pos, with_labels=False, arrows=True)
    #nx.draw_networkx_labels(G, pos, labels=node_labels, font_size=10)
    nx.draw(G, pos, with_labels=True, arrows=True)
    
    edge_labels = {(u,v): d['weight'] for u,v,d in G.edges(data=True)}
    nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels, font_size=10)
    plt.show()
    
if __name__ == "__main__":
    G = nx.DiGraph()
    filename = 'weighted-edges-inverted.txt'
    filename = 'random_tree.txt'
    filename = 'random_tree.json'
    #filename = 'input.json'
    nodes = read_tree_file(filename)

    plot_graph(nodes)
