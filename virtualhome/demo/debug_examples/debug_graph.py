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

def record_graph(comm, prefix: str, script: list, target_classes: list):
    data_dir = "../../unity_output"
    # Clean up previous images
    image_dir = os.path.join(data_dir, prefix)
    if not os.path.exists(image_dir):
        os.makedirs(image_dir)
    for root, _, files in os.walk(image_dir):
        for file in files:
            filepath = os.path.join(root, file)
            os.remove(filepath)
    
    comm.add_character('chars/Female2', initial_room='bathroom')
    success, graph = comm.environment_graph()
    
    if len(script) == 0:
        script = generate_walk_find_script(graph, target_classes)
        script_path = os.path.join(image_dir, "script_generated.txt")
        with open(script_path, "w") as f:
            for line in script:
                f.write(line + "\n")
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
        input_path=data_dir, 
        prefix=prefix, 
        output_path=os.path.join(data_dir, prefix)
    )
    
    return True



def inspect(comm):
    script_path = "/robodata/taijing/benchmarks/virtualhome/unity_output/scene4_754ab231d3_0/script_generated.txt"
    with open(script_path, "r") as f:
        script = [line.strip() for line in f.readlines() if line.strip()]
    
    record_graph(comm, prefix="test", script=script, target_classes=["book"])

if __name__ == "__main__":
    comm = UnityCommunication(port="8080")
    comm.timeout_wait = 300
    
    graph_path = "/robodata/taijing/benchmarks/virtualhome/unity_output/scene4_754ab231d3_0/0/graph.json"
    with open(graph_path, "r") as f:
        graph = json.load(f)
    if graph is None:
        raise ValueError(f"Failed to load graph from {graph_path}")
    
    comm.reset(4)
    success, message = comm.expand_scene(graph)
    if not success:
        print(f"Failed to load scene from {graph_path} to the simulator:", message)
        sys.exit(1)
        
    inspect(comm)
    
    