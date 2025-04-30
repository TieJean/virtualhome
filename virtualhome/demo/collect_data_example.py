from tqdm import tqdm
import sys, os

sys.path.append('../simulation')
from unity_simulator.comm_unity import UnityCommunication
from unity_simulator import utils_viz
from utils_demo import *
from graph_utils import *

# TODO move to config
surfaces = ["bathroomcounter", "bed", "bookshelf", "chair", "desk", "kitchencounter", "kitchentable", "sofa", "towelrack"]
ambiguous_manipulable_objects = ["book", "mug", "plate", "dishbowl", "pillow", "clothespile", "towel", "folder"]

def augment_graph(original_graph, verbose: bool = False, seed: int = 42):
    relationships = load_relationships("config/relationships.txt")
    random.seed(seed)
    
    graph = original_graph.copy()
    
    graph = remove_nodes_by_class(graph, ambiguous_manipulable_objects)
    
    surface_node_ids, surface_nodes, surface_edges = find_nodes_and_edges_by_class(graph, surfaces, verbose=verbose)
    
    # Step 3: From nodes (objects on surfaces)
    from_ids = extract_from_ids(surface_edges)
    from_nodes = extract_nodes_by_ids(graph['nodes'], from_ids)
    
    if verbose:
        unique_classes = get_unique_class_names(from_nodes)
        print("\n=== Unique object classes ON surfaces ===")
        print(unique_classes)
        
        categorized_from_nodes = categorize_from_nodes(from_nodes)
        print("\n=== Class Categorization ===")
        for k, v in categorized_from_nodes.items():
            print(f"{k}: {sorted(v)}")
            
    # Step 4: Pretty-print edge descriptions
    id_to_node = {node['id']: node for node in graph['nodes']}

    edge_descriptions = []
    for edge in surface_edges:
        from_node = id_to_node.get(edge['from_id'])
        to_node = id_to_node.get(edge['to_id'])
        if from_node and to_node:
            desc = f"{from_node['prefab_name']} ON {to_node['prefab_name']}"
            edge_descriptions.append(desc)
            
    # Step 5: Group from_ids by each surface to_id
    surface_to_objects = defaultdict(list)
    for edge in surface_edges:
        surface_to_objects[edge['to_id']].append(edge['from_id'])

    # Print summary if verbose
    if verbose:
        print("\n=== Object counts per surface ===")
        for surface_id in surface_node_ids:
            count = len(surface_to_objects.get(surface_id, []))
            class_name = id_to_node[surface_id]['class_name']
            print(f"Surface {class_name} (ID {surface_id}): {count} object(s) ON it")
            
    ambiguous_manipulable_df = get_ambiguous_manipulable_metadata(sample=True, seed=seed)
    # Step 6: Augment the graph with ambiguous manipulable objects
    next_id = 1000  # or find max used id and increment from there

    # Index surfaces by class_name for fast lookup
    surface_class_to_nodes = defaultdict(list)
    for node in surface_nodes:
        surface_class_to_nodes[node['class_name']].append(node)

    # Build surface id → room id mapping from INSIDE edges
    surface_id_to_room_id = {}
    for edge in graph['edges']:
        if edge['relation_type'] == 'INSIDE':
            surface_id_to_room_id[edge['from_id']] = edge['to_id']

    for _, row in ambiguous_manipulable_df.iterrows():
        obj_class = row["Object Name"]
        prefab_name = row["Prefab Name"]

        # Step 1: sample one surface class
        possible_surfaces = relationships.get(obj_class, {}).get("ON", [])
        if not possible_surfaces:
            if verbose:
                print(f"⚠️ No ON surfaces listed for class '{obj_class}', skipping.")
            continue
        surface_class = random.choice(possible_surfaces)

        # Step 2: find all surface nodes of that class
        candidate_surfaces = surface_class_to_nodes.get(surface_class, [])
        if not candidate_surfaces:
            if verbose:
                print(f"⚠️ No surface nodes found for class '{surface_class}', skipping.")
            continue

        # Step 3: sample one surface node
        surface_node = random.choice(candidate_surfaces)
        surface_id = surface_node['id']
        room_id = surface_id_to_room_id.get(surface_id)

        if room_id is None:
            if verbose:
                print(f"⚠️ Could not find room for surface ID {surface_id}, skipping.")
            continue

        # Step 4: create and add new object node
        new_node = {
            'id': next_id,
            'class_name': obj_class,
            'prefab_name': prefab_name,
            'category': 'AmbiguousObject',  # optionally: set a custom category
            'properties': ['GRABBABLE'],
            'states': []
        }
        add_node(graph, new_node)

        # Step 5: add ON and INSIDE edges
        add_edge(graph, fr_id=next_id, rel='ON', to_id=surface_id)
        add_edge(graph, fr_id=next_id, rel='INSIDE', to_id=room_id)

        if verbose:
            print(f"✅ Added {prefab_name} (class: {obj_class}) ON {surface_class} (id: {surface_id}), in room {room_id}")

        next_id += 1
    
    if verbose:
        _, ambig_nodes, ambig_edges = find_nodes_and_edges_by_class(graph, ambiguous_manipulable_objects, verbose=verbose)
    
    return graph


def container_nodes(graph):
    containers = ["bathroomcabinet", "cabinet", "kitchencabinet"]
    pass

if __name__ == "__main__":
    viz = False
    debug_dir = "../../outputs"
    scene_ids = [4]

    comm = UnityCommunication(port="8080")
    comm.timeout_wait = 300 

    views = []
    for scene_id in tqdm(scene_ids):
        prefix = "test"
        
        comm.reset(scene_id)
        
        if viz:
            # We will go over the line below later
            comm.remove_terrain()
            init_top_view = get_scene_cameras(comm, [-scene_id])
            
        success, graph = comm.environment_graph()
        augmented_graph = augment_graph(graph)
        success, message = comm.expand_scene(graph)
            
        if viz:
            final_top_view = get_scene_cameras(comm, [-scene_id])
            view_pil = display_grid_img(init_top_view+final_top_view, nrows=1)
            view_pil.save(os.path.join(debug_dir, f"scene_{scene_id}.png"))
        
        comm.add_character('chars/Female2', initial_room='bathroom')
        comm.add_character_camera(position=[1.5, 1.0, 0.0], rotation=[0, -60, 0], field_view=90, name="observer_camera")
        
        script = generate_walk_find_script(augmented_graph, ambiguous_manipulable_objects)
        print(script)
        
        import pdb; pdb.set_trace()
        script = script[2:3]
        
        # sofa = find_nodes(graph, class_name='sofa')[-1]
        # script = ['<char0> [Walk] <sofa> ({})'.format(sofa['id']),
        #         '<char0> [Sit] <sofa> ({})'.format(sofa['id'])]
        
        success, message = comm.render_script(script=script,
                                            processing_time_limit=120,
                                            find_solution=False,
                                            image_width=640,
                                            image_height=480,  
                                            skip_animation=False,
                                            recording=True,
                                            save_pose_data=True,
                                            camera_mode=["observer_camera"],
                                            file_name_prefix=prefix)
        print("Finish Rendering")

        input_path = os.path.abspath('../../unity_output/')
        output_path = os.path.abspath('../../outputs/')
        utils_viz.generate_video(input_path=input_path, prefix=prefix, output_path=output_path)
    
    exit(0)