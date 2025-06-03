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
        
    # - sofa (ID: 27) is in livingroom (ID: 11)
    # - chair (ID: 28) is in livingroom (ID: 11)
    # - desk (ID: 29) is in livingroom (ID: 11)
    # - nightstand (ID: 35) is in livingroom (ID: 11)
    # - rug (ID: 83) is in livingroom (ID: 11)
    # - rug (ID: 84) is in livingroom (ID: 11)
    # - chair (ID: 134) is in kitchen (ID: 111)
    # - chair (ID: 135) is in kitchen (ID: 111)
    # - chair (ID: 136) is in kitchen (ID: 111)
    # - chair (ID: 137) is in kitchen (ID: 111)
    # - kitchentable (ID: 138) is in kitchen (ID: 111)
    # - bookshelf (ID: 140) is in kitchen (ID: 111)
    # - sofa (ID: 141) is in kitchen (ID: 111)
    # - kitchencounter (ID: 150) is in kitchen (ID: 111)
    # - stove (ID: 154) is in kitchen (ID: 111)
    # - plate (ID: 182) is in kitchen (ID: 111)
    # - plate (ID: 185) is in kitchen (ID: 111)
    # - plate (ID: 186) is in kitchen (ID: 111)
    # - rug (ID: 196) is in kitchen (ID: 111)
    # - rug (ID: 231) is in bedroom (ID: 214)
    # - rug (ID: 239) is in bedroom (ID: 214)
    # - radio (ID: 251) is in bedroom (ID: 214)
    # - chair (ID: 254) is in bedroom (ID: 214)
    # - sofa (ID: 272) is in bedroom (ID: 214)
    # - nightstand (ID: 273) is in bedroom (ID: 214)
    # - bed (ID: 274) is in bedroom (ID: 214)
    # - cabinet (ID: 275) is in bedroom (ID: 214)
    # - bathroomcabinet (ID: 297) is in bathroom (ID: 276)
    # - bathroomcounter (ID: 298) is in bathroom (ID: 276)
    # - towelrack (ID: 301) is in bathroom (ID: 276)
    # - towelrack (ID: 341) is in bathroom (ID: 276)
    # - rug (ID: 362) is in bathroom (ID: 276)
    # - rug (ID: 363) is in bathroom (ID: 276)