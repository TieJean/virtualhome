from tqdm import tqdm
import sys, os
import json
import argparse

sys.path.append('../simulation')
from unity_simulator.comm_unity import UnityCommunication
from unity_simulator import utils_viz
from utils_demo import *
from graph_utils import *
from viz_utils import *

scene_and_remove_ids = {
    4: [31, 34, 132, 252]
}

def parse_args():
    parser = argparse.ArgumentParser(description='Generate script for virtual home')
    parser.add_argument("--output_dir", type=str, default="example_graphs", help="Directory to save the generated script")
    return parser.parse_args()

def generate_graph(comm, scene_id: int):
    if scene_id not in scene_and_remove_ids.keys():
        return
    remove_ids = scene_and_remove_ids[scene_id]
    
    comm.reset(scene_id)
    success, graph = comm.environment_graph()
    
    if not success:
        raise RuntimeError("Failed to get environment graph")
    remove_all_objects_on_surfaces(graph, ["wallshelf"])
    remove_nodes_by_ids(graph, remove_ids)
    success, message = comm.expand_scene(graph)
    if not success:
        print("Failed to expand scene:", message)
        return False
    
    success, graph = comm.environment_graph()
    graph_path = os.path.join(args.output_dir, f"scene{scene_id}_graph.json")
    with open(graph_path, "w") as f:
        json.dump(graph, f, indent=2)

if __name__ == "__main__":
    args = parse_args()
    os.makedirs(args.output_dir, exist_ok=True)
    
    comm = UnityCommunication(port="8080")
    comm.timeout_wait = 300
    
    for scene_id in tqdm(range(10)):
        generate_graph(comm, scene_id)