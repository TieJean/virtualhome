
from tqdm import tqdm
import sys, os
import json
import argparse
import copy
from typing import List, Dict, Any
import random
import numpy as np
import time

sys.path.append('../simulation')
from unity_simulator.comm_unity import UnityCommunication
from unity_simulator import utils_viz
from utils_demo import *
from graph_utils import *
from viz_utils import *
import hashlib

def parse_args():
    parser = argparse.ArgumentParser(description='Collect data for virtual home')
    parser.add_argument('--script_dir', type=str, default="example_scripts", help='Directory containing scripts')
    parser.add_argument('--scene_ids', nargs='+', type=int, default=[4], help='List of scene IDs to collect data from')
    parser.add_argument("--graph_dir", type=str, default="example_graphs", help="Directory containing scene graphs")
    parser.add_argument('--clean_surfaces', nargs='+', type=str, default=[], help='List of surfaces to clean')
    parser.add_argument('--clean_containers', nargs='+', type=str, default=[], help='List of containers to clean')
    parser.add_argument('--clean_classes', nargs='+', type=str, default=["pillow", "book", "toy", "magazine", "folder"], help='List of target classes to replace')
    parser.add_argument('--clean_ids', nargs='+', type=int, default=[], help='List of target IDs to replace')
    parser.add_argument('--n_runs_per_scene', type=int, default=6, help="Number of runs per scene")
    parser.add_argument('--seed', type=int, default=40, help='Random seed')
    parser.add_argument('--port', type=str, required=True, help='Port for Unity communication')
    return parser.parse_args()

def _record_graph(comm, save_dir: str, prefix: str, script: List[str], robot_initial_state = None) -> bool:
    image_dir = os.path.join(save_dir, prefix)
    if not os.path.exists(image_dir):
        os.makedirs(image_dir)
        
    # Clear previous images
    for root, _, files in os.walk(image_dir):
        for file in files:
            filepath = os.path.join(root, file)
            os.remove(filepath)
            
    if robot_initial_state is not None:
        comm.add_character('chars/Female2', position=robot_initial_state["initial_position"], initial_room="bathroom")
    else:
        comm.add_character('chars/Female2', initial_room="bathroom")
    time.sleep(5) # NOTE this is necessary to ensure a fixed starting pose
    
    success, graph = comm.environment_graph()
    if not success:
        print("Failed to get environment graph:", graph)
        return False
    
    batch_size = 10
    for start in range(0, len(script), batch_size):
        sub_script = script[start:start + batch_size]
        success, message = comm.render_script(script=sub_script,
                                            processing_time_limit=6000,
                                            find_solution=False,
                                            image_width=640,
                                            image_height=480,  
                                            skip_animation=False,
                                            recording=True,
                                            save_pose_data=True,
                                            camera_mode=["FIRST_PERSON"],
                                            image_synthesis=["normal", "seg_inst", "seg_class", "depth"],
                                            file_name_prefix=prefix)
    
        if not success:
            import pdb; pdb.set_trace()
            raise RuntimeError(f"Failed to render script: {message}")
    output_dir = os.path.join(save_dir, prefix, "0")
    
    # Save the agent graph and environment graph
    agent_graph_path = os.path.join(output_dir, "agent_graph.json") # This is necessary to obtain ground truth
    graph_path = os.path.join(output_dir, "graph.json")
    isinstance_colors_path = os.path.join(output_dir, "instance_colors.json")
    
    success, instance_colors = comm.instance_colors()
    if not success:
        print("Failed to get instance colors:", instance_colors)
        return False
    try:
        with open(isinstance_colors_path, 'w') as f:
            json.dump(instance_colors, f, indent=2)
    except Exception as e:
        print(f"Failed to save instance colors: {e}")
        return False
    
    success, agent_graph = comm.environment_graph()
    try:
        with open(agent_graph_path, 'w') as f:
            json.dump(agent_graph, f, indent=2)
    except Exception as e:
        print(f"Failed to save agent graph: {e}")
        return False
    
    graph = remove_nodes_by_classes(agent_graph, ["character"])
    success, message = comm.expand_scene(graph)
    if not success:
        print("Failed to expand scene:", message)
        return False
    success, graph = comm.environment_graph()
    try:
        with open(graph_path, 'w') as f:
            json.dump(graph, f, indent=2)
    except Exception as e:
        print(f"Failed to save environment graph: {e}")
        return False
    
    utils_viz.generate_video(
        input_path=args.data_dir, 
        prefix=prefix, 
        output_path=os.path.join(args.data_dir, prefix)
    )
    return True

def _replace_objects(args, 
                     comm, 
                     scene_id, 
                     verbose: bool = True):
    _prepare_scene(args, comm, scene_id)
    time.sleep(1)  # Ensure the scene is ready
    
    _, orginal_graph = comm.environment_graph()
    
    _, graph = comm.environment_graph()
    success, graph, placement_log = place_all_objects(graph, 
                                             args.prefab_classes, 
                                             args.class_placements, 
                                             relations=("INSIDE"),
                                             verbose=verbose)
    
    success, expand_message = comm.expand_scene(graph)
    
    if not success:
        print("Failed to expand scene after placing objects:", expand_message)
        
        comm.reset(scene_id)
        success, message = comm.expand_scene(orginal_graph)
        
        remove_ids = []
        
        if isinstance(expand_message, dict) and "unplaced" in expand_message:
            for item in expand_message["unplaced"]:
                # Extract the node id after the dot, e.g., 'folder.730' -> 730
                try:
                    node_id = int(item.split(".")[-1])
                    remove_ids.append(node_id)
                except Exception:
                    pass
            graph = remove_nodes_by_ids(graph, remove_ids)
            success, message = comm.expand_scene(graph)
            # Remove entries from placement_log whose node id is in remove_ids
            placement_log = [entry for entry in placement_log if entry[2] not in remove_ids]
            
            if not success:
                print("Failed to expand scene after removing unplaced objects:", message)
                return False, None
        else:
            print("Failed to expand scene after placing objects:", message)
            return False, None
    
    return True, placement_log
    
def _prepare_scene(args, comm, scene_id: int):
    comm.reset(scene_id)
    
    _, graph = comm.environment_graph()
    graph = remove_nodes_by_classes(graph, args.clean_classes)
    success, message = comm.expand_scene(graph)
    if not success:
        raise RuntimeError(f"Failed to expand scene: {message}")
    
    _, graph = comm.environment_graph()
    graph = remove_nodes_by_classes(graph, [args.prefab_classes.keys()])
    success, message = comm.expand_scene(graph)
    if not success:
        raise RuntimeError(f"Failed to expand scene: {message}")
    
    _, graph = comm.environment_graph()
    graph = remove_all_objects_on_surfaces(graph, args.clean_surfaces, relations=("ON"))
    success, message = comm.expand_scene(graph)
    if not success:
        raise RuntimeError(f"Failed to expand scene: {message}")
    
    _, graph = comm.environment_graph()
    graph = remove_all_objects_on_surfaces(graph, args.clean_containers, relations=("INSIDE"), verbose=True)
    success, message = comm.expand_scene(graph)
    if not success:
        raise RuntimeError(f"Failed to expand scene: {message}")
    
    _, graph = comm.environment_graph()
    graph = remove_all_objects_on_surfaces_by_ids(graph, args.clean_ids)
    success, message = comm.expand_scene(graph)
    if not success:
        raise RuntimeError(f"Failed to expand scene: {message}")
    
    _, graph = comm.environment_graph()
    graph = remove_nodes_by_ids(graph, args.clean_ids)
    success, message = comm.expand_scene(graph)
    if not success:
        raise RuntimeError(f"Failed to expand scene: {message}")

def run_once(args, comm, script: List[str], robot_initial_state, prefix: str):
    print(f"Running script with prefix: {prefix}")
    success, placement_log = _replace_objects(args, comm, scene_id, verbose=True)
    # import pdb; pdb.set_trace()
    if not success:
        return False
    
    time.sleep(1)  # Ensure the scene is ready after placing objects
    _, graph = comm.environment_graph()
    script = generate_walk_find_script(graph, ["toy", "book", "magazine", "folder"],)
    
    if not _record_graph(comm, args.data_dir, prefix, script, robot_initial_state):
        return False
    
    obj_placement_savepath = os.path.join(args.data_dir, prefix, "0", "object_placement.csv")
    
    header = [
        "obj_cls", "obj_prefab_name", "obj_node_id",
        "surface_cls", "surface_prefab_name", "surface_id",
        "room_cls", "room_prefab_name", "room_id"
    ]
    os.makedirs(os.path.dirname(obj_placement_savepath), exist_ok=True)
    with open(obj_placement_savepath, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerows(placement_log)
    placement_log
    
    return True
    

def collect_data_in_one_scene(args, comm, scene_id: int):
    
    # robot_script_path = os.path.join(args.script_dir, f"scene{scene_id}_robot_script.txt")
    # with open(robot_script_path, "r") as f:
    #     script = [line.strip() for line in f if line.strip()]
    # if script is None or len(script) == 0:
    #     raise ValueError(f"No script found for scene {scene_id} in {robot_script_path}")
    script = None
    
    # robot_initial_state_path = os.path.join(args.script_dir, f"scene{scene_id}_robot_initial_state.json")
    # with open(robot_initial_state_path, "r") as f:
    #     robot_initial_state = json.load(f)
    # if robot_initial_state is None or "initial_position" not in robot_initial_state or "initial_lookat" not in robot_initial_state:
    #     raise ValueError(f"No initial state found for scene {scene_id} in {robot_initial_state_path}")
    
    for i_run in tqdm(range(args.n_runs_per_scene), desc=f"Scene {scene_id}"):
        run_once(args, comm, script, None, prefix=f"test")
        time.sleep(5)  # Ensure there's a delay between runs
    
if __name__ == "__main__":
    args = parse_args()
    args.data_dir = os.path.abspath('../../unity_output/')
    os.makedirs(args.data_dir, exist_ok=True)
    
    random.seed(args.seed)
    np.random.seed(args.seed)
    
    comm = UnityCommunication(port=args.port)
    comm.timeout_wait = 60000
    
    prefab_classes = {
        "book": ["Book_13", "Book_18", "Book_27"],
        "toy": ["Toy_10", "Toy_5", "Toy_2"],
        "folder": ["Folder_1", "Folder_2", "Folder_3"],
        "magazine": ["Magazine_7l", "Magazine_7p", "Magazine_4"],
    }
    args.prefab_classes = {k.replace("_", "").lower(): v for k, v in prefab_classes.items()}
    
    with open("../resources/object_script_placing_customed.json", "r") as f:
        class_placements = json.load(f)
    # Normalize keys and destinations
    normalized_class_placements = {}
    for cls_name, placements in class_placements.items():
        new_key = cls_name.replace("_", "").lower()
        new_placements = []
        for entry in placements:
            new_entry = entry.copy()
            if 'destination' in new_entry:
                new_entry['destination'] = new_entry['destination'].replace("_", "").lower()
            new_placements.append(new_entry)
        normalized_class_placements[new_key] = new_placements
    args.class_placements = normalized_class_placements
    
    for scene_id in args.scene_ids:
        collect_data_in_one_scene(args, comm, scene_id)