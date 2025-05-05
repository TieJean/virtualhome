from tqdm import tqdm
import sys, os
import json

sys.path.append('../simulation')
from unity_simulator.comm_unity import UnityCommunication
from unity_simulator import utils_viz
from utils_demo import *
from graph_utils import *

# TODO move to config
surfaces = ["bathroomcounter", "bed", "bookshelf", "chair", "desk", "kitchencounter", "kitchentable", "sofa", "towelrack", "wallshelf", "coffeetable", "dinningtable", "kitchentable"]
# ambiguous_manipulable_objects = ["book", "dishbowl", "pillow", "clothespile", "clothesshirt", "clothespant" "towel", "folder"]
ambiguous_manipulable_objects = ["book", "folder"]

def augment_graph(comm, verbose: bool = False, seed: int = 42, max_retries: int = 10):
    random.seed(seed)
    relationships = load_relationships("config/relationships.txt")
    ambiguous_manipulable_df = get_ambiguous_manipulable_metadata(ambiguous_manipulable_objects, sample=True, seed=seed)
    
    success, graph = comm.environment_graph()

    if not success:
        raise RuntimeError("Failed to get initial environment graph.")
    
    graph = remove_nodes_by_class(graph, ambiguous_manipulable_objects)
    # graph = remove_all_objects_on_surfaces(graph, surfaces, verbose) # TODO: optional?
    success, message = comm.expand_scene(graph)  # Clean baseline
    if not success:
        import pdb; pdb.set_trace()
        raise RuntimeError("Failed to expand scene after removing objects.")
    success, graph = comm.environment_graph()

    _, surface_nodes, _ = find_nodes_and_edges_by_class(graph, surfaces, verbose=verbose)
    surface_class_to_nodes = defaultdict(list)
    for node in surface_nodes:
        surface_class_to_nodes[node['class_name']].append(node)

    # Map surface node ID → room ID
    surface_id_to_room_id = {
        edge['from_id']: edge['to_id']
        for edge in graph['edges'] if edge['relation_type'] == 'INSIDE'
    }

    start_id = sorted([n["id"] for n in graph["nodes"]])[-1] + 1
    next_id = start_id
    failed_objects = []

    for _, row in ambiguous_manipulable_df.iterrows():
        obj_class = row["Object Name"]
        prefab_name = row["Prefab Name"]
        possible_surfaces = relationships.get(obj_class, {}).get("ON", [])
        placed = False

        for attempt_i in range(max_retries):
            success, graph = comm.environment_graph()
            if not possible_surfaces:
                if verbose:
                    print(f"⚠️ No ON surfaces listed for class '{obj_class}', skipping.")
                break

            surface_class = random.choice(possible_surfaces)
            candidates = surface_class_to_nodes.get(surface_class, [])
            if not candidates:
                if verbose:
                    print(f"⚠️ No surface nodes found for surface class '{surface_class}', skipping attempt.")
                continue

            surface_node = random.choice(candidates)
            surface_id = surface_node['id']
            room_id = surface_id_to_room_id.get(surface_id)
            if room_id is None:
                if verbose:
                    print(f"⚠️ Could not find room for surface ID {surface_id}, skipping attempt.")
                continue

            # Build and inject
            new_node = {
                'id': next_id,
                'class_name': obj_class,
                'prefab_name': prefab_name,
                'category': 'AmbiguousObject',
                'properties': ['GRABBABLE'],
                'states': []
            }
            add_node(graph, new_node)
            add_edge(graph, fr_id=next_id, rel='ON', to_id=surface_id)
            add_edge(graph, fr_id=next_id, rel='INSIDE', to_id=room_id)
            
            success, message = comm.expand_scene(graph)
            if success:
                # sorted([n["id"] for n in graph["nodes"]])[-1]
                if verbose:
                    print(f"✅ Added {prefab_name} (class: {obj_class}; id: {next_id}) ON {surface_class} (id: {surface_id}), in room {room_id}")
                next_id += 1
                placed = True
                break
            else:
                # Clean up and retry
                # graph['nodes'] = [n for n in graph['nodes'] if n['id'] != next_id]
                # graph['edges'] = [e for e in graph['edges'] if e['from_id'] != next_id and e['to_id'] != next_id]
                if verbose:
                    surface_desc = surface_node.get('prefab_name') or surface_node['class_name']
                    print(f"❌ Failed to place {prefab_name} ON {surface_desc} (id: {surface_id}) on attempt {attempt_i+1}/{max_retries}, trying again...")

        if not placed:
            failed_objects.append((obj_class, prefab_name))
            if verbose:
                print(f"⚠️ Gave up on placing {prefab_name} (class: {obj_class}) after {max_retries} attempts.")

    if failed_objects:
        print("\n⚠️ Some objects could not be placed:")
        for cls, prefab in failed_objects:
            print(f"   - {prefab} (class: {cls})")
            
    # === Construct minimal GT graph for added ambiguous objects ===
    success, graph = comm.environment_graph()
    ambiguous_node_ids = list(range(start_id, next_id))
    gt_nodes = []
    gt_edges = []
    id_set = set()
    id_to_node = {n['id']: n for n in graph['nodes']}

    for node_id in ambiguous_node_ids:
        node = id_to_node[node_id]
        gt_nodes.append(node)
        id_set.add(node_id)

        for edge in graph['edges']:
            if edge['from_id'] == node_id or edge['to_id'] == node_id:
                gt_edges.append(edge)
                other_id = edge['to_id'] if edge['from_id'] == node_id else edge['from_id']
                if other_id not in id_set:
                    id_set.add(other_id)
                    if other_id in id_to_node:
                        gt_nodes.append(id_to_node[other_id])

    gt_graph = {
        "nodes": gt_nodes,
        "edges": gt_edges
    }

    return graph, gt_graph


def container_nodes(graph):
    containers = ["bathroomcabinet", "cabinet", "kitchencabinet"]
    pass

if __name__ == "__main__":
    scenepath = "../../unity_output/test/0/graph.json"
    # scenepath = None
    verbose = True
    viz = False
    debug_dir = "../../outputs"
    scene_ids = [4]

    comm = UnityCommunication(port="8080")
    comm.timeout_wait = 300 

    views = []
    for scene_id in tqdm(scene_ids):
        # prefix = "test"
        prefix = "test_observer_camera"
        
        comm.reset(scene_id)
        
        if viz:
            # We will go over the line below later
            comm.remove_terrain()
            init_top_view = get_scene_cameras(comm, [-scene_id])
            
            
        # comm.add_character('chars/Female2', initial_room='bathroom')
        # comm.add_character_camera(position=[1.5, 1.0, 0.0], rotation=[0, -60, 0], field_view=90, name="observer_camera")
        # script = ['<char0> [Walk] <wallshelf> (31)', '<char0> [Find] <book> (88)']
        # success, message = comm.render_script(script=script, processing_time_limit=120, find_solution=False, image_width=640, image_height=480, skip_animation=False, recording=True, save_pose_data=True, camera_mode=["observer_camera"], file_name_prefix=prefix)
        # import pdb; pdb.set_trace()
        
        if scenepath is not None:
            with open(scenepath, "r") as f:
                graph = json.load(f)
            success, message = comm.expand_scene(graph)
            if not success:
                raise RuntimeError(f"Failed to expand scene after loading graph at: {scenepath}.")
        else:
            graph, gt_graph = augment_graph(comm, verbose=verbose)
            print("here")
            output_dir = os.path.join("../../unity_output", prefix, "0")
            os.makedirs(output_dir, exist_ok=True)
            graph_path = os.path.join(output_dir, "graph.json")
            with open(graph_path, "w") as f:
                json.dump(graph, f, indent=2)
                if verbose:
                    print(f"graph saved to {graph_path}")
            graph_gt_path = os.path.join(output_dir, "gt_graph.json")
            with open(graph_gt_path, "w") as f:
                json.dump(gt_graph, f, indent=2)
                if verbose:
                    print(f"GT graph saved to {graph_gt_path}")
                
        success, graph = comm.environment_graph()
            
        if viz:
            final_top_view = get_scene_cameras(comm, [-scene_id])
            view_pil = display_grid_img(init_top_view+final_top_view, nrows=1)
            view_pil.save(os.path.join(debug_dir, f"scene_{scene_id}.png"))
        
        comm.add_character('chars/Female2', initial_room='bathroom')
        # comm.add_character_camera(position=[1.0, 1.0, -0.5], rotation=[0, -45, 0], field_view=90, name="observer_camera")
        comm.add_character_camera(position=[0.5, 1.5, -1.0], rotation=[0, -45, 0], field_view=90, name="observer_camera")
        
        script = generate_walk_find_script(graph, ambiguous_manipulable_objects)
        print(script)
        
        # script = script[5:10]
        
        # sofa = find_nodes(graph, class_name='sofa')[-1]
        # script = ['<char0> [Walk] <sofa> ({})'.format(sofa['id']),
        #         '<char0> [Sit] <sofa> ({})'.format(sofa['id'])]
        # success, message = comm.render_script(script=script, processing_time_limit=120, find_solution=False, image_width=640, image_height=480, skip_animation=False, recording=True, save_pose_data=True, camera_mode=["observer_camera"], file_name_prefix=prefix)
        
        success, message = comm.render_script(script=script,
                                            processing_time_limit=300,
                                            find_solution=False,
                                            image_width=640,
                                            image_height=480,  
                                            skip_animation=False,
                                            recording=True,
                                            save_pose_data=True,
                                            camera_mode=["observer_camera"],
                                            # camera_mode=["FIRST_PERSON"],
                                            file_name_prefix=prefix)
        import pdb; pdb.set_trace()
        print("Finish Rendering")

        input_path = os.path.abspath('../../unity_output/')
        output_path = os.path.abspath('../../outputs/')
        utils_viz.generate_video(input_path=input_path, prefix=prefix, output_path=output_path)
    
    exit(0)