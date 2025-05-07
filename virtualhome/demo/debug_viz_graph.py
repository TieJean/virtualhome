from tqdm import tqdm
import sys, os
import json

sys.path.append('../simulation')
from unity_simulator.comm_unity import UnityCommunication
from unity_simulator import utils_viz
from utils_demo import *
from graph_utils import *
from viz_utils import *

ambiguous_manipulable_objects = ["book", "dishbowl", "pillow", "clothespile", "clothesshirt", "clothespant", "towel", "folder"]

if __name__ == "__main__":
    seed = 42
    verbose = False
    viz = False
    debug_dir = "../../outputs"
    relationships = load_relationships("config/relationships.txt")
    scene_id = 4
    prefix = "test"
    
    random.seed(seed)
    
    comm = UnityCommunication(port="8080")
    comm.timeout_wait = 300 
    
    comm.reset(scene_id)
    
    scenepath_in = f"../../outputs/scene_{scene_id}_modified.json"
    with open(scenepath_in, "r") as f:
        graph = json.load(f)
    print(f"Loaded graph from: {scenepath_in}")
    success, message = comm.expand_scene(graph, animate_character=True)
    if not success:
        raise RuntimeError(f"Failed to expand scene after loading graph at: {scenepath_in}.")
    print(f"Expanded scene with graph from: {scenepath_in}")
    
    comm.add_character('chars/Female2', initial_room='bathroom')
    success, graph = comm.environment_graph()
    if not success:
        import pdb; pdb.set_trace()
        raise RuntimeError(f"Failed to get environment graph after expanding scene.")
    
    script = generate_walk_find_script(graph, ["book"])
    print(script)
    
    success, message = comm.render_script(script=script,
                            processing_time_limit=1000,
                            find_solution=False,
                            image_width=640,
                            image_height=480,  
                            skip_animation=False,
                            recording=True,
                            save_pose_data=True,
                            camera_mode=["observer_camera"],
                            file_name_prefix="test")
    
    import pdb; pdb.set_trace()
    
    input_path = os.path.abspath('../../unity_output/')
    output_path = os.path.abspath('../../outputs/')
    utils_viz.generate_video(input_path=input_path, prefix=prefix, output_path=output_path)