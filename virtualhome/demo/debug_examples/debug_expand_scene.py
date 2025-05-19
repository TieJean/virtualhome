from tqdm import tqdm
import sys, os
import json

sys.path.append('../simulation')
from unity_simulator.comm_unity import UnityCommunication
from unity_simulator import utils_viz
sys.path.append(".")
from utils_demo import *
from graph_utils import *

ambiguous_manipulable_objects = ["book", "dishbowl", "pillow", "clothespile", "clothesshirt", "clothespant", "towel", "folder"]

def example_1(comm):
    ### Case 1 ###
    comm.reset(3)
    success, graph = comm.environment_graph()
    comm.reset(4)
    success, message = comm.expand_scene(graph)
    print("[Case 1]", message) # Error processing input graph: Object reference not set to an instance of an object.
    
    ### Case 2 ###
    comm.reset(3)
    success, graph = comm.environment_graph()
    comm.reset()
    success, message = comm.expand_scene(graph)
    print("[Case 2]", message) # {}
    
    ### Case 3 ###
    comm.reset()
    with open("example_graphs/TestScene1_graph.json", "r") as f:
        graph = json.load(f)
    success, message = comm.expand_scene(graph)
    print("[Case 3]", message) # Error processing input graph: Object reference not set to an instance of an object.    
    
    ### Case 4 ###
    comm.reset(4)
    success, graph = comm.environment_graph()
    comm.add_character('chars/Female2')
    success, message = comm.expand_scene(graph)
    print("[Case 4]", message) # {}
    
    ### Case 5 ###
    comm.reset(4)
    success, original_graph = comm.environment_graph()
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
    print("[Case 5.1]", message) # {}
    
    comm.add_character('chars/Female2')
    s, g = comm.environment_graph()
    cat_id = [node['id'] for node in g['nodes'] if node['class_name'] == 'cat'][0]
    
    script = ['<char0> [Walk] <sofa> ({})'.format(sofa['id']),
          '<char0> [Find] <cat> ({})'.format(cat_id),
          '<char0> [Grab] <cat> ({})'.format(cat_id),
          '<char0> [Sit] <sofa> ({})'.format(sofa['id'])]
    
    # success, graph = comm.environment_graph()
    # success, message = comm.expand_scene(graph) 
    # print("[Case 5.2]", message) # Abort. In simulator - KeyNotFoundException: The given key 'Female2 (UnityEngine.GameObject)' was not present in the dictionary.
    
    graph = remove_nodes_by_classes(g, ["character"])
    success, message = comm.expand_scene(graph)
    print("[Case 5.3]", message) # {}  
    
    with open("example_graphs/TestScene4.json", "w") as f:
        json.dump(graph, f, indent=2)
    graph = json.load(open("example_graphs/TestScene4.json", "r"))
    comm.reset()
    success, message = comm.expand_scene(graph)
    print("[Case 5.4]", message) # {}
    
    success, message = comm.expand_scene(original_graph)
    print("[Case 5.5]", message) # {}
    
    ### Case 6 ###
    comm.reset(4)
    _, ambiguous_manipulable_nodes, _ = find_nodes_and_edges_by_class(graph, ambiguous_manipulable_objects, verbose=False)
    graph = remove_duplicate_prefabs_by_class(graph, ambiguous_manipulable_objects, verbose=False)
    success, message = comm.expand_scene(graph)
    print("[Case 6]", message) # {}

if __name__ == "__main__":
    comm = UnityCommunication(port="8080")
    comm.timeout_wait = 300 
    
    example_1(comm)
    
    