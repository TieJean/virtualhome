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

scene_to_waypoints = {
    4: [("livingroom", "wallshelf"), 
        ("livingroom", "sofa"),
        ("livingroom", "wallshelf"), 
        ("kitchen", "bookshelf"), 
        ("kitchen", "wallshelf"), 
        ("kitchen", "sofa"), 
        ("bedroom", "sofa"), 
        ("bedroom", "wallshelf"), 
       ],
}

def parse_args():
    parser = argparse.ArgumentParser(description='Generate script for virtual home')
    parser.add_argument("--graph_dir", type=str, default=None, help="Directory containing scene graphs")
    parser.add_argument("--output_dir", type=str, default="example_scripts", help="Directory to save the generated script")
    return parser.parse_args()

def generate_script(comm, scene_id: int):
    if scene_id not in scene_to_waypoints:
        return
    
    if args.graph_dir is not None:
        graph_path = os.path.join(args.graph_dir, f"scene{scene_id}_graph.json")
        if os.path.exists(graph_path):
            with open(graph_path, "r") as f:
                print("Loading graph from file:", graph_path)
                graph = json.load(f)
                success, message = comm.expand_scene(graph)
                if not success:
                    print("Failed to expand scene:", message)
                    return False
        else:
            comm.reset(scene_id)
    else:
        comm.reset(scene_id)
        
    # Dynamically find surface node IDs based on room/surface pairs
    target_ids = []
    for room_name, surface_class in scene_to_waypoints[scene_id]:
        surface_nodes = find_objects_in_room(graph, room_name, surface_class)
        target_ids.extend(node['id'] for node in surface_nodes)
        
    comm.add_character('chars/Male2', initial_room='bathroom')
    success, graph = comm.environment_graph()
    if not success:
        raise RuntimeError("Failed to get environment graph")
    script = generate_fixed_waypoint_script(graph, target_ids)
    success, graph = comm.environment_graph()
    graph = remove_nodes_by_classes(graph, ["character"])
    success, message = comm.expand_scene(graph)
    if not success:
        print("Failed to expand scene:", message)
        return False
    
    output = os.path.join(args.output_dir, f"robot_scene_{scene_id}_script.txt")
    with open(output, "w") as f:
        for line in script:
            f.write(line + "\n")

if __name__ == "__main__":
    args = parse_args()
    os.makedirs(args.output_dir, exist_ok=True)
    
    comm = UnityCommunication(port="8080")
    comm.timeout_wait = 300
    
    for scene_id in tqdm(range(10)):
        generate_script(comm, scene_id)