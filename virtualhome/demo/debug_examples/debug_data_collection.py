from tqdm import tqdm
import sys, os
import json

sys.path.append('../simulation')
from unity_simulator.comm_unity import UnityCommunication
from unity_simulator import utils_viz
sys.path.append(".")
from utils_demo import *
from graph_utils import *

ambiguous_manipulable_objects = ["book", "folder", "dishbowl", "pillow", "clothespile", "clothesshirt", "clothespant", "towel", "folder"]


def example_expand_scene(comm):
    comm.reset(4)
    success, graph = comm.environment_graph()
    sofa = find_nodes(graph, class_name='sofa')[-1]
    add_node(graph, {'class_name': 'cat', 
                    'category': 'Animals',
                    "prefab_name": "Cat_1", 
                    'id': 1000, 
                    'properties': [], 
                    'states': []})
    add_edge(graph, 1000, 'ON', sofa['id'])
    add_node(graph, {'class_name': 'cat', 
                    'category': 'Animals',
                    "prefab_name": "Cat_2", 
                    'id': 1001, 
                    'properties': [], 
                    'states': []})
    add_edge(graph, 1001, 'ON', sofa['id'])
    success, message = comm.expand_scene(graph)
    print("[Case 1.1]", message) # {}
    
    success, graph = comm.environment_graph()
    cat_id = [node['id'] for node in graph['nodes'] if node['class_name'] == 'cat'][0]
    
    script = ['<char0> [Walk] <sofa> ({})'.format(sofa['id']),
          '<char0> [Find] <cat> ({})'.format(cat_id),
          '<char0> [Grab] <cat> ({})'.format(cat_id),
          '<char0> [Sit] <sofa> ({})'.format(sofa['id'])]
    comm.add_character('chars/Female2')
    success, message = comm.render_script(script=script,
                                        processing_time_limit=500,
                                        find_solution=False,
                                        image_width=640,
                                        image_height=480,  
                                        skip_animation=False,
                                        recording=True,
                                        save_pose_data=True,
                                        camera_mode=["FIRST_PERSON"],
                                        file_name_prefix="test")
    print("render_script success: ", success)
    
    success, graph = comm.environment_graph()
    graph = remove_nodes_by_classes(graph, ["character"])
    success, message = comm.expand_scene(graph)
    print("[Case 1.2]", message) #  {}
    
    with open("example_graphs/TestScene4.json", "w") as f:
        json.dump(graph, f, indent=2)
    graph = json.load(open("example_graphs/TestScene4.json", "r"))
    comm.reset()
    success, message = comm.expand_scene(graph)
    print("[Case 1.3]", message) # {'unplaced': ['cat.368']} - This is expected
    
def example_generate_walk_find_script(comm):
    graph = json.load(open("example_graphs/TestScene4.json", "r"))
    comm.reset()
    success, message = comm.expand_scene(graph)
    print("[Case 2.1]", message) # {'unplaced': ['cat.368']} - This is expected
    
    success, graph = comm.environment_graph()
    comm.add_character('chars/Female2', initial_room='bathroom')
    script = generate_walk_find_script(graph, ["towel"])
    success, message = comm.render_script(script=script,
                                        processing_time_limit=1000,
                                        find_solution=False,
                                        image_width=640,
                                        image_height=480,  
                                        skip_animation=False,
                                        recording=True,
                                        save_pose_data=True,
                                        camera_mode=["FIRST_PERSON"],
                                        file_name_prefix="test")
    print("render_script success: ", success)
    success, graph = comm.environment_graph()
    graph = remove_nodes_by_classes(graph, ["character"])
    success, message = comm.expand_scene(graph)
    print("[Case 2.2]", message) #  {}
    if success:
        input_path = os.path.abspath('../../unity_output/')
        output_path = os.path.abspath('../../outputs/')
        utils_viz.generate_video(input_path=input_path, prefix="test", output_path=output_path)
    
    with open("example_graphs/TestScene4_generate_script.json", "w") as f:
        json.dump(graph, f, indent=2)
    graph = json.load(open("example_graphs/TestScene4_generate_script.json", "r"))
    comm.reset()
    success, message = comm.expand_scene(graph)
    print("[Case 2.3]", message) # {}
    
def example_single_object_replacement(comm):
    input_path = os.path.abspath('../../unity_output/')
    output_path = os.path.abspath('../../outputs/')
    relationships = load_relationships("config/relationships.txt")
    prefix = "test"
    image_dir = os.path.join('../../unity_output/', prefix)
    if not os.path.exists(image_dir):
        os.makedirs(image_dir)
    for root, _, files in os.walk(image_dir):
        for file in files:
            filepath = os.path.join(root, file)
            os.remove(filepath)
    
    comm.reset(4)
    random.seed(40)
    
    success, graph_before = comm.environment_graph()
    success, graph = comm.environment_graph()
    
    graph = replace_prefab_names(graph, "book", ["PRE_PRO_Book_08", "PRE_PRO_Book_07", "PRE_PRO_Book_01"])
    # NOTE: For some reason, the books are not visually distinct??
    
    comm.reset()
    success, message = comm.expand_scene(graph)
    print("[Case 3.1]", message) # {}
    
    comm.add_character('chars/Female2', initial_room='bathroom')
    success, graph = comm.environment_graph()
    _, ambiguous_manipulable_nodes, _ = find_nodes_and_edges_by_class(graph, ambiguous_manipulable_objects, verbose=False)
    nodes = filter_nodes_by_class(ambiguous_manipulable_nodes, ["book"])
    for node in nodes:
        script = generate_single_object_replacement_script(graph, node, relationships, verbose=False)
        # print(script)
        success, message = comm.render_script(script=script,
                                processing_time_limit=1000,
                                find_solution=False,
                                image_width=640,
                                image_height=480,  
                                skip_animation=True, # False,
                                recording=False, # True,
                                save_pose_data=True,
                                file_name_prefix=prefix)
        print("render_script success: ", success)
        
    success, graph = comm.environment_graph()
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
                                        camera_mode=["FIRST_PERSON"],
                                        file_name_prefix=prefix)
    print("render_script success: ", message)
    
    success, graph = comm.environment_graph()
    graph = remove_nodes_by_classes(graph, ["character"])
    success, message = comm.expand_scene(graph)
    print("[Case 3.2]", message) #  {}
    if success:
        utils_viz.generate_video(input_path=input_path, prefix=prefix, output_path=output_path)

    success, graph_after = comm.environment_graph()
    diff_node_edges(graph_before, graph_after)
    
    with open("example_graphs/TestScene4_generate_script.json", "w") as f:
        json.dump(graph, f, indent=2)
    graph = json.load(open("example_graphs/TestScene4_generate_script.json", "r"))
    comm.reset()
    success, message = comm.expand_scene(graph)
    print("[Case 3.3]", message) # {}
    

if __name__ == "__main__":
    comm = UnityCommunication(port="8080")
    comm.timeout_wait = 300
    
    # example_expand_scene(comm)
    # example_generate_walk_find_script(comm)
    example_single_object_replacement(comm)