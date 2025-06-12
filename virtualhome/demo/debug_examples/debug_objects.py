from tqdm import tqdm
import sys, os
import json
import argparse
import copy

sys.path.append('../simulation')
from unity_simulator.comm_unity import UnityCommunication
from unity_simulator import utils_viz
sys.path.append(".")
from utils_demo import *
from graph_utils import *

if __name__ == "__main__":
    comm = UnityCommunication(port="8080")
    comm.timeout_wait = 300
    
    graph_path = "example_graphs/scene4_graph.json"
    with open(graph_path, "r") as f:
        graph = json.load(f)
    if graph is None:
        raise ValueError(f"Failed to load graph from {graph_path}")
    
    target_class = ["dishbowl", "cupcake", "milkshake", "pancake", "pie", "poundcake", "pudding"]
    find_nodes_and_edges_by_class(graph, target_class, verbose=True)