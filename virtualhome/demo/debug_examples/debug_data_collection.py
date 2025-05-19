from tqdm import tqdm
import sys, os
import json

sys.path.append('../simulation')
from unity_simulator.comm_unity import UnityCommunication
from unity_simulator import utils_viz
sys.path.append(".")
from utils_demo import *
from graph_utils import *

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
    
    success, graph_test = comm.environment_graph()
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
    

if __name__ == "__main__":
    comm = UnityCommunication(port="8080")
    comm.timeout_wait = 300
    
    example_expand_scene(comm)
    example_generate_walk_find_script(comm)