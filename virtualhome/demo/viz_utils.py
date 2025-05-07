import sys, os
import json

sys.path.append('../simulation')
from unity_simulator import utils_viz
from utils_demo import *
from graph_utils import *

def viz_graph(comm, path: str, target_classes: list, scene_id: int = None, prefix: str = "test", verbose: bool = False):
    """
    Visualize the graph in the Unity environment.
    
    Args:
        comm (UnityCommunication): The Unity communication object.
        path (str): Path to the graph file.
        target_classes (list): List of target classes to visualize.
        verbose (bool): If True, print additional information.
    """
    with open(path, "r") as f:
        graph = json.load(f)
        
    if graph is None:
        raise RuntimeError(f"Failed to load graph from: {path}.")
    print(f"Loaded graph from: {path}.")
    
    if scene_id is not None:
        comm.reset(scene_id)
    print(f"Reset scene with ID: {scene_id}.")
    success, message = comm.expand_scene(graph)
    if not success:
        raise RuntimeError(f"Failed to expand scene after loading graph at: {path}.")
    print(f"Expanded scene with graph: {path}.")
    
    comm.add_character('chars/Female2', initial_room='bathroom')
    
    success, graph = comm.environment_graph()
    script = generate_walk_find_script(graph, target_classes)
    print(script)
    script = script[:1]
    success, message = comm.render_script(script=script,
                                        processing_time_limit=1000,
                                        find_solution=False,
                                        image_width=640,
                                        image_height=480,  
                                        skip_animation=False,
                                        recording=True,
                                        save_pose_data=True,
                                        camera_mode=["observer_camera"],
                                        file_name_prefix=prefix)
    
    
    if not success:
        raise RuntimeError(f"Failed to render script: {message}.")
    
    input_path = os.path.abspath('../../unity_output/')
    output_path = os.path.abspath('../../outputs/')
    utils_viz.generate_video(input_path=input_path, prefix=prefix, output_path=output_path)