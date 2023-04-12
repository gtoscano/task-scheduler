#!/usr/bin/env python
# -*- coding: utf-8 -*-

import networkx as nx
import matplotlib.pyplot as plt
import random
import json


def store_graph(G, filename):
    # Save the graph to a file
    dependencies = []
    max_value = -1
    nodes = []
    for u, v, w in G.edges(data=True):
        if u > max_value: max_value = u
        if v > max_value: max_value = v
        dependencies.append([u, v])
        if u not in nodes:
            nodes.append(u)
    seconds = []
    tasks = {}
    print(max_value)
    print(nodes)
    for i in range(max_value+1):
        if i in nodes:
            print('jeje: ', i)
        seconds_value = 0 if i not in nodes else random.randint(1, 10)
        print(seconds_value)
        seconds.append(seconds_value)
        tasks[str(i)] = seconds[-1]
    
    with open(filename+'.json', 'w') as f:
        json.dump({'tasks':tasks, 
                   'dependencies':dependencies}, f)


    with open(filename+'.txt', 'w') as f:
        counter = 0
        for u, v, w in G.edges(data=True):
            new_seconds = seconds[counter]
            counter += 1
            f.write(f"{u} {v} {new_seconds}\n")



def draw_tree(tree):
    pos = nx.nx_agraph.graphviz_layout(tree, prog='dot')
    nx.draw(tree, pos, with_labels=True, node_size=500, node_color="skyblue", font_size=12, arrows=True)
    plt.show()

def random_directed_tree(num_nodes, root=None):
    # Generate a random undirected tree
    undirected_tree = nx.random_tree(num_nodes)
    
    # Convert the undirected tree to a directed tree
    directed_tree = nx.DiGraph(undirected_tree)

    # Optional: Set a specific root and re-orient edges from the root
    if root is not None:
        directed_tree = nx.bfs_tree(directed_tree, root)

    return directed_tree

# Parameters
num_nodes = 100
root = 0  # Optional: specify the root node
filename = 'random_tree'

# Generate a random directed tree
random_directed_tree = random_directed_tree(num_nodes, root).reverse()
# Draw the directed tree
draw_tree(random_directed_tree)
store_graph(random_directed_tree, filename)

# Check if the graph is a directed tree
if nx.is_directed_acyclic_graph(random_directed_tree) and nx.is_tree(random_directed_tree.to_undirected()):
    print("The graph is a directed tree")
else:
    print("The graph is not a directed tree")

