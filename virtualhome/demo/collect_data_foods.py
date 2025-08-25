from tqdm import tqdm
import sys, os
import json
import argparse
import tempfile
import copy
from typing import List, Dict, Any, Optional, Tuple, Union
import random
import numpy as np
import time
import shutil
import csv

sys.path.append('../simulation')
from unity_simulator.comm_unity import UnityCommunication
from unity_simulator import utils_viz
from utils_demo import *
from graph_utils import *
from viz_utils import *

def parse_args():
    parser = argparse.ArgumentParser(description='Collect data for virtual home')
    parser.add_argument('--script_dir', type=str, default="example_scripts", help='Directory containing scripts')
    parser.add_argument('--scene_ids', nargs='+', type=int, default=[4], help='List of scene IDs to collect data from')
    parser.add_argument("--graph_dir", type=str, default="example_graphs", help="Directory containing scene graphs")
    parser.add_argument('--clean_surfaces', nargs='+', type=str, default=[], help='List of surfaces to clean')
    parser.add_argument('--clean_classes', nargs='+', type=str, default=["dishbowl"], help='List of target classes to replace')
    parser.add_argument('--clean_ids', nargs='+', type=int, default=[], help='List of target IDs to replace')
    parser.add_argument('--n_runs_per_scene', type=int, default=16, help="Number of runs per scene")
    parser.add_argument('--seed', type=int, default=40, help='Random seed')
    parser.add_argument('--port', type=str, required=True, help='Port for Unity communication')
    return parser.parse_args()

def _roll_episode_dirs(base_dir: str, prefix: str, verbose: bool = False) -> str:
    """
    Under {base_dir}/{prefix}, delete folder '0' (if exists) and move '1' to '0'.
    Returns the absolute path to the new '0' folder.
    """
    root = os.path.join(base_dir, prefix)
    dir0 = os.path.join(root, "0")
    dir1 = os.path.join(root, "1")

    if not os.path.isdir(dir1):
        raise FileNotFoundError(f"Expected folder not found: {dir1}")

    if os.path.isdir(dir0):
        if verbose: print(f"🧹 Removing existing: {dir0}")
        shutil.rmtree(dir0, ignore_errors=True)

    # atomic rename on same filesystem
    if verbose: print(f"🔀 Moving {dir1} → {dir0}")
    os.replace(dir1, dir0)
    return dir0

def _swap_character_token(
    scripts: Union[List[str], List[List[str]]],
    src: str = "char0",
    dst: str = "char1",
    inplace: bool = False,
) -> Union[List[str], List[List[str]]]:
    """
    Replace all occurrences of <src> with <dst> in script lines.

    - Accepts a flat list of lines or a list of lists.
    - By default returns a new list; set inplace=True to modify inner lists directly.

    Example:
        swap_character_token(lines, "char0", "char1")
    """
    src_tok = f"<{src}>"
    dst_tok = f"<{dst}>"

    def _replace(line: str) -> str:
        return line.replace(src_tok, dst_tok)

    if scripts and isinstance(scripts[0], list):
        # list-of-lists
        if inplace:
            for block in scripts:  # modifies in place
                for i, line in enumerate(block):
                    block[i] = _replace(line)
            return scripts
        else:
            return [[_replace(line) for line in block] for block in scripts]
    else:
        # flat list
        if inplace:
            for i, line in enumerate(scripts):  # modifies in place
                scripts[i] = _replace(line)
            return scripts
        else:
            return [_replace(line) for line in scripts]

def _record_graph(args, comm, save_dir: str, prefix: str, script: List[str], robot_initial_state = None) -> bool:
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

    script = _swap_character_token(script, "char0", "char1")
    batch_size = 8
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
    
    output_dir = os.path.join(save_dir, prefix, "1")
    
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
                     verbose: bool = False):
    
    
    reset_scene_from_saved_graph(comm, args.saved_graph_path)
    time.sleep(1)  # Ensure the scene is ready
    
    success, graph = comm.environment_graph()
    scripts, placement_log, _ = generate_random_placement_scripts(graph, 
                                                                  args.target_classes, 
                                                                  args.class_placements, 
                                                                  relations = ["ON"])

    comm.add_character('chars/Female2', initial_room="bathroom")
    time.sleep(5)
    script = ['<char0> [Walk] <kitchen> (111)']
    success, message = comm.render_script(script=script, find_solution=False, skip_animation=True, recording=False, save_pose_data=False)
    if not success:
        import pdb; pdb.set_trace()
        raise RuntimeError(f"Failed to render script: {message}")
    
    for script in scripts:
        success, message = comm.render_script(script=script, find_solution=False, skip_animation=False, recording=False)
        if not success:
            import pdb; pdb.set_trace()
            print("Failed to render script:", message)
            return False, None
        time.sleep(1)  # Ensure the scene is ready after placing objects
        
    success, agent_graph = comm.environment_graph()
    if not success:
        raise RuntimeError(f"Failed to get agent graph: {message}")
    graph = remove_nodes_by_classes(agent_graph, ["character"])
    success, message = comm.expand_scene(graph)
    if not success:
        print("Failed to expand scene:", message)
        return False, None
    
    return True, placement_log


def prepare_scene_and_save_graph(
    args,
    comm,
    scene_id: int,
    tmp_path: Optional[str] = None,
    verbose: bool = False,
) -> Tuple[bool, Optional[str]]:
    """
    Prepare the scene (clean objects/surfaces as in your original _prepare_scene),
    then save the resulting environment graph to `tmp_path` (tempfile if None).

    Returns:
        (success, saved_path)
    """
    # Always start from a clean reset of the scene
    comm.reset(scene_id)

    # 1) Remove classes listed in args.clean_classes
    ok, graph = comm.environment_graph()
    if not ok: 
        raise RuntimeError("Failed to get environment graph (step 1).")
    clean_classes = args.clean_classes
    graph = remove_nodes_by_classes(graph, clean_classes)
    ok, msg = comm.expand_scene(graph)
    if not ok:
        raise RuntimeError(f"Failed to expand scene after removing clean_classes: {msg}")

    # 3) Clean objects on specified surfaces (accept ON/INSIDE)
    ok, graph = comm.environment_graph()
    if not ok:
        if verbose: print("Failed to get environment graph (step 3).")
        return False, None
    clean_surfaces = args.clean_surfaces
    graph = remove_all_objects_on_surfaces(graph, args.clean_surfaces, relations=("ON", "INSIDE"))
    ok, msg = comm.expand_scene(graph)
    if not ok:
        raise RuntimeError(f"Failed to expand scene after removing objects on surfaces: {msg}")

    # 4) Remove objects on specific surface IDs (if any)
    ok, graph = comm.environment_graph()
    if not ok:
        if verbose: print("Failed to get environment graph (step 4a).")
        return False, None
    if getattr(args, "clean_ids", None):
        graph = remove_all_objects_on_surfaces_by_ids(graph, args.clean_ids, verbose=verbose)
        ok, msg = comm.expand_scene(graph)
        if not ok:
            raise RuntimeError(f"Failed to expand scene after removing objects on surface IDs: {msg}")

    # 5) Remove nodes by IDs (final)
    ok, graph = comm.environment_graph()
    if not ok:
        if verbose: print("Failed to get environment graph (step 4b).")
        return False, None
    if getattr(args, "clean_ids", None):
        graph = remove_nodes_by_ids(graph, args.clean_ids)
        ok, msg = comm.expand_scene(graph)
        if not ok:
            raise RuntimeError(f"Failed to expand scene after removing nodes by IDs: {msg}")

    # 6) Fetch the final prepared graph and save to tmp JSON
    ok, graph = comm.environment_graph()
    if not ok:
        raise ValueError("Failed to get final prepared environment graph.")
    
    if tmp_path is None:
        # put the temp file in args.data_dir if available; else system temp dir
        base_dir = getattr(args, "data_dir", None)
        if base_dir:
            os.makedirs(base_dir, exist_ok=True)
            fd, tmp_path = tempfile.mkstemp(prefix="prepared_scene_", suffix=".json", dir=base_dir)
            os.close(fd)
        else:
            fd, tmp_path = tempfile.mkstemp(prefix="prepared_scene_", suffix=".json")
            os.close(fd)

    with open(tmp_path, "w") as f:
        json.dump(graph, f, indent=2)
    if verbose:
        print(f"✅ Prepared scene saved to: {tmp_path}")
    return tmp_path

def reset_scene_from_saved_graph(comm, saved_graph_path: str) -> bool:
    """
    Load a previously saved environment graph JSON and expand the scene to this state.
    """
    if not os.path.isfile(saved_graph_path):
        raise FileNotFoundError(f"Saved graph file not found: {saved_graph_path}")

    with open(saved_graph_path, "r") as f:
        graph = json.load(f)
    if not graph:
        raise ValueError(f"Invalid graph data in: {saved_graph_path}")

    ok, msg = comm.expand_scene(graph)
    if not ok:
        raise RuntimeError(f"Failed to expand scene from saved graph: {msg}")

def run_once(args, comm, script: List[str], robot_initial_state, prefix: str, scene_id: int):
    print(f"Running script with prefix: {prefix}")
    success, placement_log = _replace_objects(args, comm, scene_id, verbose=True)
    print(placement_log)
    if not success:
        return False
    
    time.sleep(1)  # Ensure the scene is ready after placing objects
    
    if not _record_graph(args, comm, args.data_dir, prefix, script, robot_initial_state):
        return False
    
    out_dir = _roll_episode_dirs(args.data_dir, prefix, verbose=True)

    obj_placement_savepath = os.path.join(out_dir, "object_placement.csv")

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
    
    robot_script_path = os.path.join(args.script_dir, f"scene{scene_id}_robot_script.txt")
    with open(robot_script_path, "r") as f:
        script = [line.strip() for line in f if line.strip()]
    if script is None or len(script) == 0:
        raise ValueError(f"No script found for scene {scene_id} in {robot_script_path}")
    
    robot_initial_state_path = os.path.join(args.script_dir, f"scene{scene_id}_robot_initial_state.json")
    with open(robot_initial_state_path, "r") as f:
        robot_initial_state = json.load(f)
    if robot_initial_state is None or "initial_position" not in robot_initial_state or "initial_lookat" not in robot_initial_state:
        raise ValueError(f"No initial state found for scene {scene_id} in {robot_initial_state_path}")
    
    for i_run in tqdm(range(args.n_runs_per_scene), desc=f"Scene {scene_id}"):
        run_once(args, comm, script, robot_initial_state, prefix=f"scene{scene_id}_{i_run:02d}_foods", scene_id=scene_id)
        time.sleep(5)  # Ensure there's a delay between runs
        
if __name__ == "__main__":
    args = parse_args()
    
    args.data_dir = os.path.abspath('../../unity_output/')
    os.makedirs(args.data_dir, exist_ok=True)
    
    random.seed(args.seed)
    np.random.seed(args.seed)
    
    comm = UnityCommunication(port=args.port)
    comm.timeout_wait = 60000
    
    # comm.add_character('chars/Female2', initial_room="bathroom")
    # script = ['<char0> [Walk] <kitchen> (111)']
    # success, message = comm.render_script(script=script, find_solution=False, skip_animation=True, recording=False, save_pose_data=False)
    # import pdb; pdb.set_trace()
    
    for scene_id in args.scene_ids:
        
        args.saved_graph_path = prepare_scene_and_save_graph(args, comm, scene_id, verbose=True)
        
        success, graph = comm.environment_graph()
        (class_list, counts) = get_classes_by_category(graph, "Foods", True)
        for cls, cnt in counts.items():
            if cnt != 1:
                raise ValueError(f"Class {cls} has {cnt} instances; expected exactly 1.")
        args.target_classes = class_list
    
        with open(f"../resources/object_script_placing_customed.json", "r") as f:
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
    
        collect_data_in_one_scene(args, comm, scene_id)