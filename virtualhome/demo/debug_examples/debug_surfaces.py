import json
import sys

sys.path.append('../simulation')
from unity_simulator.comm_unity import UnityCommunication
from unity_simulator import utils_viz
sys.path.append(".")
from utils_demo import *
from graph_utils import *

if __name__ == "__main__":
    ### All possible surfaces
    prefab_file = "../resources/PrefabClassCustomed.json"
    with open(prefab_file, "r") as f:
        prefab_data = json.load(f)['prefabClasses']
    property_file = "../resources/properties_data_unity.json"
    with open(property_file, "r") as f:
        property_data = json.load(f)
    classname_equiv_file = "../resources/class_name_equivalence.json"
    with open(classname_equiv_file, "r") as f:
        classname_equivalences = json.load(f)
        
    surface_objects_data = {
        k: v for k, v in property_data.items() if "SURFACES" in v
    }
    #'bathroomcabinet', 'bathroomcounter', 'bed', 'bench', 'boardgame', 
    # 'bookshelf', 'chair', 'cabinet', 'coffeetable', 'cuttingboard', 
    # 'desk', 'diningtable', 'plate', 'fryingpan', 'kitchencounter', 
    # 'kitchentable', 'mousemat', 'nightstand', 'radio', 'rug', 'sofa', 
    # 'stove', 'towelrack'
    
    ### Surface objects within the scene
    comm = UnityCommunication(port="8080")
    comm.timeout_wait = 300
    
    with open("../../unity_output/scene4_754ab231d3_0/0/graph.json", "r") as f:
        graph = json.load(f)
    if graph is None:
        raise ValueError("Graph is None, please check the file path or content.")
    comm.reset(4)
    success, message = comm.expand_scene(graph)
    if not success:
        raise RuntimeError(f"Failed to expand scene: {message}")
    
    ### Extract surface nodes from the graph
    surface_nodes = [
        node for node in graph["nodes"]
        if node["class_name"].lower() in surface_objects_data
    ]

    # Build quick lookup for id -> node
    id_to_node = {node["id"]: node for node in graph["nodes"]}

    print(f"Found {len(surface_nodes)} surface nodes in the scene:\n")

    for node in surface_nodes:
        room_name = "Unknown"
        room_id = "?"
        for edge in graph["edges"]:
            if edge["from_id"] == node["id"] and edge["relation_type"] == "INSIDE":
                room_node = id_to_node.get(edge["to_id"], {})
                if room_node.get("category") == "Rooms":
                    room_name = room_node.get("class_name", "Unknown")
                    room_id = room_node.get("id", "?")
                    break
        print(f" - {node['class_name']} (ID: {node['id']}) is in {room_name} (ID: {room_id})")
        
        
    import pdb; pdb.set_trace()