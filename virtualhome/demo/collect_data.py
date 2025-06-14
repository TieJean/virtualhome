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
import hashlib

def parse_args():
    parser = argparse.ArgumentParser(description='Collect data for virtual home')
    parser.add_argument('--script_dir', type=str, default="example_scripts", help='Directory containing scripts')
    parser.add_argument("--graph_dir", type=str, default="example_graphs", help="Directory containing scene graphs")
    parser.add_argument('--target_classes', nargs='+', type=str, default=["book"], help='List of target ambiguous manipulable object classes')
    parser.add_argument('--seed', type=int, default=40, help='Random seed')
    return parser.parse_args()

def get_dataset_name(ambiguous_objects, scene_id):
    obj_str = ','.join(sorted(ambiguous_objects))
    hash_input = f"{obj_str}:{scene_id}"
    hash_digest = hashlib.md5(hash_input.encode('utf-8')).hexdigest()
    return f"scene{scene_id}_{hash_digest[:10]}"

def prepare_scene(args, comm, scene_id: int):
    random.seed(args.seed)
    if args.graph_dir is not None:
        graph_path = os.path.join(args.graph_dir, f"scene{scene_id}_graph.json")
        if os.path.exists(graph_path):
            with open(graph_path, "r") as f:
                print("Loading graph from file:", graph_path)
                graph = json.load(f)
                
                comm.reset(scene_id)
                success, message = comm.expand_scene(graph)
                if not success:
                    print("Failed to expand scene:", message)
                    return False
        else:
            comm.reset(scene_id)
    else:
        comm.reset(scene_id)
    
    success, graph = comm.environment_graph()
    graph = remove_nodes_by_classes(graph, args.target_classes)
    success, message = comm.expand_scene(graph)
    if not success:
        print("Failed to expand scene:", message)
        return False
    
    graph = remove_all_objects_on_surfaces(
        graph, 
        # ["desk", "wallshelf", "kitchentable", "kitchencounter", "bathroomcounter"]
        ["desk", "wallshelf", "kitchentable", "plate"]
    )
    success, message = comm.expand_scene(graph)
    if not success:
        print("Failed to expand scene:", message)
        return False
    
    for target_class in args.target_classes:
        success, graph = comm.environment_graph()
        graph = insert_object_with_placement(graph, args.prefab_classes, args.class_placements, target_class, relations=["ON"], n=2, verbose=True)
        success, message = comm.expand_scene(graph)
        if not success:
            print("Failed to expand scene:", message)
            return False
        
    return True

def replace_objects(args, comm, verbose:bool = False):
    _, graph = comm.environment_graph()
    
    class_to_prefabs = {}
    for target_class in args.target_classes:
        prefab_names = [
            node["prefab_name"]
            for node in graph["nodes"]
            if node["class_name"].lower() == target_class.lower()
        ]
        class_to_prefabs[target_class] = prefab_names
        
    graph = remove_nodes_by_classes(graph, args.target_classes)
    success, message = comm.expand_scene(graph)
    if not success:
        print("Failed to expand scene:", message)
        return False
    
    success, graph = comm.environment_graph()
    for target_class in args.target_classes:
        graph = insert_object_with_placement(
            graph, 
            args.prefab_classes, 
            args.class_placements, 
            target_class, 
            relations=["ON"], 
            prefab_candidates=class_to_prefabs[target_class],
            n=3, 
            verbose=verbose
        )
        success, message = comm.expand_scene(graph)
        if not success:
            print("Failed to expand scene:", message)
            return False
    
    return True
        
def record_graph(args, comm, prefix: str, script: list):
    # Clean up previous images
    image_dir = os.path.join(args.data_dir, prefix)
    if not os.path.exists(image_dir):
        os.makedirs(image_dir)
    for root, _, files in os.walk(image_dir):
        for file in files:
            filepath = os.path.join(root, file)
            os.remove(filepath)
    
    comm.add_character('chars/Female2', initial_room='bathroom')
    success, graph = comm.environment_graph()
    print("prefix: ", prefix)
    success, message = comm.render_script(script=script,
                                        processing_time_limit=2000,
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
        print("Failed to render script:", message)
        return False
    
    success, graph = comm.environment_graph()
    graph = remove_nodes_by_classes(graph, ["character"])
    success, message = comm.expand_scene(graph)
    if not success:
        print("Failed to expand scene:", message)
        return False
    utils_viz.generate_video(
        input_path=args.data_dir, 
        prefix=prefix, 
        output_path=os.path.join(args.data_dir, prefix)
    )
    
    return True

def run_once(args, comm, scene_id: int):
    if not prepare_scene(args, comm, scene_id):
        print(f"Failed to prepare scene {scene_id}.")
        return
    import pdb; pdb.set_trace()
    
    graphs = []
    _, graph = comm.environment_graph()
    graphs.append(graph)
    
    # for i in range(1, 6): # TODO
    #     if not replace_objects(args, comm):
    #         continue
    #     _, graph = comm.environment_graph()
    #     graphs.append(graph)
    
    dataset_name = get_dataset_name(args.target_classes, scene_id)
    
    script_path = os.path.join(args.script_dir, f"robot_scene_{scene_id}_script.txt")
    with open(script_path, "r") as f:
        script = [line.strip() for line in f if line.strip()]
    
    for i, graph in enumerate(graphs):
        prefix = f"{dataset_name}_{i}" # TODO
        # prefix = "test"
        comm.reset(scene_id)
        success, message = comm.expand_scene(graph)
        if not success:
            print("Failed to expand scene:", message)
            continue
        record_graph(args, comm, prefix, script)
        subgraph_gt = extract_minimal_subgraph_by_classes(graph, args.target_classes)
        
        output_dir = os.path.join(args.data_dir, prefix, "0")
        graph_path = os.path.join(output_dir, "graph.json")
        with open(graph_path, "w") as f:
            json.dump(graph, f, indent=2)        
        graph_gt_path = os.path.join(output_dir, "graph_gt.json")
        with open(graph_gt_path, "w") as f:
            json.dump(subgraph_gt, f, indent=2)
            
        class_to_prefabs = {}
        for class_name in args.target_classes:
            prefab_names = [
                node["prefab_name"]
                for node in graph["nodes"]
                if node["class_name"].lower() == class_name.lower()
            ]
            class_to_prefabs[class_name] = prefab_names
        
        class_prefabs_path = os.path.join(output_dir, "target_objects.txt")
        with open(class_prefabs_path, "w") as f:
            for cls, prefabs in class_to_prefabs.items():
                f.write(','.join([cls] + prefabs) + "\n")

if __name__ == "__main__":
    args = parse_args()
    
    args.data_dir = os.path.abspath('../../unity_output/')
    os.makedirs(args.data_dir, exist_ok=True)
    
    with open("../resources/PrefabClassCustomed.json", "r") as f:
        prefab_classes = {}
        for prefab_class in json.load(f)["prefabClasses"]:
            prefab_classes[prefab_class["className"].lower()] = prefab_class["prefabs"]
    normalized_prefab_classes = {
        class_name.replace("_", "").lower(): prefabs
        for class_name, prefabs in prefab_classes.items()
    }
        
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
    
    args.prefab_classes = prefab_classes
    args.class_placements = normalized_class_placements
    
    comm = UnityCommunication(port="8080")
    comm.timeout_wait = 300
    
    run_once(args, comm, scene_id=4)