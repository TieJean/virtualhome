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

scene_and_target_ids = {
    4: [29, 32, 33, 34, 139, 140, 252],
}

def parse_args():
    parser = argparse.ArgumentParser(description='Generate script for virtual home')
    parser.add_argument("--output_dir", type=str, default="example_scripts", help="Directory to save the generated script")
    return parser.parse_args()

def generate_script(comm, scene_id: int):
    if scene_id not in scene_and_target_ids.keys():
        return
    target_ids = scene_and_target_ids[scene_id]
    
    comm.reset(scene_id)
    success, graph = comm.environment_graph()
    if not success:
        raise RuntimeError("Failed to get environment graph")
    
    script = generate_fixed_waypoint_script(graph, target_ids)
    
    output = os.path.join(args.output_dir, f"robot_scene_{scene_id}_script.txt")
    with open(output, "w") as f:
        for line in script:
            f.write(line + "\n")

if __name__ == "__main__":
    args = parse_args()
    os.makedirs(args.output_dir, exist_ok=True)
    
    omm = UnityCommunication(port="8080")
    comm.timeout_wait = 300
    
    for scene_id in tqdm(range(10)):
        generate_script(comm, scene_id)