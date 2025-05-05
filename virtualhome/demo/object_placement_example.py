import sys, os
import random

sys.path.append('../simulation')
from unity_simulator.comm_unity import UnityCommunication
from unity_simulator import utils_viz
from utils_demo import *
from graph_utils import *

surfaces = ["bathroomcounter", "bed", "bookshelf", "chair", "desk", "kitchencounter", "kitchentable", "sofa", "towelrack", "wallshelf", "coffeetable", "dinningtable", "kitchentable"]
ambiguous_manipulable_objects = ["book", "dishbowl", "pillow", "clothespile", "clothesshirt", "clothespant", "towel", "folder"]

if __name__ == "__main__":
    scene_id = 4
    prefix = "test"
    seed = 42
    verbose = False
    
    comm = UnityCommunication(port="8080")
    comm.timeout_wait = 300 
    
    # Step 1: Reset and load scene
    comm.reset(scene_id)

    # Step 2: Get the environment graph
    success, graph = comm.environment_graph()

    # Step 3: Add shirt node and place inside bedroom
    _, ambiguous_manipulable_nodes, _ = find_nodes_and_edges_by_class(graph, ambiguous_manipulable_objects, verbose=verbose)
    node = ambiguous_manipulable_nodes[-16] # book
    
    src_room = find_room_of_node(graph, node['id'])
    if not src_room:
        raise RuntimeError("❌ Could not find room for object.")

    _, surface_nodes, _ = find_nodes_and_edges_by_class(graph, surfaces, verbose=verbose)
    if not surface_nodes:
        raise RuntimeError("❌ No surface found in the scene.")

    dst_surface = surface_nodes[0]
    dst_room = find_room_of_node(graph, dst_surface['id'])
    if not dst_room:
        raise RuntimeError("❌ Could not find room for surface.")

    # Step 5: Add character
    comm.add_character('chars/Female2', initial_room='bathroom')
    
    # Step 7: Generate program
    script = [
        f'<char0> [Walk] <{src_room["class_name"]}> ({src_room["id"]})',
        f'<char0> [Grab] <{node["class_name"]}> ({node["id"]})',
        f'<char0> [Walk] <{dst_room["class_name"]}> ({dst_room["id"]})',
        f'<char0> [Walk] <{dst_surface["class_name"]}> ({dst_surface["id"]})',
        f'<char0> [Put] <{node["class_name"]}> ({node["id"]}) <{dst_surface["class_name"]}> ({dst_surface["id"]})'
    ]
    print(script)
    
    success, message = comm.render_script(script=script,
                                        processing_time_limit=300,
                                        find_solution=False,
                                        image_width=640,
                                        image_height=480,  
                                        skip_animation=False,
                                        recording=True,
                                        save_pose_data=True,
                                        # camera_mode=["FIRST_PERSON"],
                                        file_name_prefix=prefix)
    import pdb; pdb.set_trace()
    
    input_path = os.path.abspath('../../unity_output/')
    output_path = os.path.abspath('../../outputs/')
    utils_viz.generate_video(input_path=input_path, prefix=prefix, output_path=output_path)