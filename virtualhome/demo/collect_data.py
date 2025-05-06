from tqdm import tqdm
import sys, os
import json

sys.path.append('../simulation')
from unity_simulator.comm_unity import UnityCommunication
from unity_simulator import utils_viz
from utils_demo import *
from graph_utils import *

ambiguous_manipulable_objects = ["book", "dishbowl", "pillow", "clothespile", "clothesshirt", "clothespant", "towel", "folder"]

if __name__ == "__main__":
    seed = 42
    # scenepath = "../../unity_output/test/0/graph.json"
    verbose = False
    viz = False
    debug_dir = "../../outputs"
    scene_ids = [4]
    relationships = load_relationships("config/relationships.txt")
    
    comm = UnityCommunication(port="8080")
    comm.timeout_wait = 300 
    
    for scene_id in tqdm(scene_ids):
        prefix = "test_collect_data"
        scenepath_in = None
        scenepath_out = None
        
        if scenepath_in is not None:
            with open(scenepath_in, "r") as f:
                graph = json.load(f)
            success, message = comm.expand_scene(graph)
            if not success:
                raise RuntimeError(f"Failed to expand scene after loading graph at: {scenepath_in}.")
        else:
            # Step 1: Reset and load scene
            comm.reset(scene_id)
        
        # Step 2: Get the environment graph
        success, graph = comm.environment_graph()
        
        if True:
            _, ambiguous_manipulable_nodes, _ = find_nodes_and_edges_by_class(graph, ambiguous_manipulable_objects, verbose=verbose)
            graph = remove_duplicate_prefabs_by_class(graph, ambiguous_manipulable_objects, verbose=True)
            
            success, message = comm.expand_scene(graph)
            if not success:
                import pdb; pdb.set_trace()
                raise RuntimeError("Failed to expand scene after removing objects.")
    
        if True:
            # Convert to DataFrame
            success, graph = comm.environment_graph()
            _, ambiguous_manipulable_nodes, _ = find_nodes_and_edges_by_class(graph, ambiguous_manipulable_objects, verbose=verbose)
            
            df = pd.DataFrame(ambiguous_manipulable_nodes)

            # Group by class_name
            class_counts = df["class_name"].value_counts()

            print("\n📊 Object counts by class_name:")
            print(class_counts)

            # Table of class_name and prefab_name
            print("\n📋 class_name ↔ prefab_name table:")
            print(df[["class_name", "prefab_name"]].sort_values(by="class_name").to_string(index=False))
        
        success, graph = comm.environment_graph()
        # save the graph to a file
        if scenepath_out is not None:
            with open(scenepath_out, "w") as f:
                json.dump(graph, f, indent=2)
        
        comm.add_character('chars/Female2', initial_room='bathroom')
        comm.add_character_camera(position=[0.0, 1.0, -0.1], rotation=[0, 0, 0], field_view=90, name="observer_camera")
        script = generate_walk_find_script(graph, ["towel"])
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
                                        file_name_prefix=prefix)
        
        print(script)
        
        import pdb; pdb.set_trace()
        
        input_path = os.path.abspath('../../unity_output/')
        output_path = os.path.abspath('../../outputs/')
        utils_viz.generate_video(input_path=input_path, prefix=prefix, output_path=output_path)