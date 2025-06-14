from collections import defaultdict
import requests
from bs4 import BeautifulSoup
import pandas as pd
import re
import random
from utils_demo import *
import json

def prepare_pano_character_camera(comm):
    s, msg = comm.add_character_camera(position=[0, 1.8,  0.0], rotation=[20,  0, 0], field_view=60, name="pano_0")
    s, msg = comm.add_character_camera(position=[0, 1.8,  0.0], rotation=[20, 60, 0], field_view=60, name="pano_1")
    s, msg = comm.add_character_camera(position=[0, 1.8,  0.0], rotation=[20, 120, 0], field_view=60, name="pano_2")
    s, msg = comm.add_character_camera(position=[0, 1.8,  0.0], rotation=[20, 180, 0], field_view=60, name="pano_3")
    s, msg = comm.add_character_camera(position=[0, 1.8,  0.0], rotation=[20, 240, 0], field_view=60, name="pano_4")
    s, msg = comm.add_character_camera(position=[0, 1.8,  0.0], rotation=[20, 300, 0], field_view=60, name="pano_5")

def viz_scene(comm, scene_id:int, savepath:str):
    view = get_scene_cameras(comm, [scene_id])
    view_pil = display_grid_img(view, nrows=1)
    if os.path.isdir(savepath):
        view_pil.save(os.path.join(savepath, f"scene_{scene_id}.png"))
    else:
        view_pil.save(savepath)

def extract_from_ids(edges):
    """Return a set of 'from_id's from a list of edges."""
    return {edge['from_id'] for edge in edges}

def extract_nodes_by_ids(nodes, id_set):
    """Return nodes whose 'id' is in the given id_set."""
    return [node for node in nodes if node['id'] in id_set]

def get_unique_class_names(nodes):
    """
    Return sorted list of unique class names from a list of nodes.
    """
    return sorted({node['class_name'] for node in nodes})

def find_nodes_and_edges_by_class(graph, target_classes: list, verbose: bool = False):
    """
    Given a graph and a list of target class_names, return:
    - nodes whose class_name is in target_classes
    - edges where from_id or to_id matches any of those nodes
    If verbose=True, prints grouped edge info by node, including class names.
    """
    node_ids = set()
    selected_nodes = []

    for node in graph['nodes']:
        if node['class_name'] in target_classes:
            selected_nodes.append(node)
            node_ids.add(node['id'])

    selected_edges = [
        edge for edge in graph['edges']
        if edge['from_id'] in node_ids or edge['to_id'] in node_ids
    ]

    if verbose:
        # Build id → class_name mapping
        id_to_class = {node['id']: node['class_name'] for node in graph['nodes']}

        print("\n=== Edges ===")
        edges_by_node = {node['id']: [] for node in selected_nodes}
        for edge in selected_edges:
            if edge['from_id'] in edges_by_node:
                edges_by_node[edge['from_id']].append(('from', edge))
            if edge['to_id'] in edges_by_node:
                edges_by_node[edge['to_id']].append(('to', edge))

        for node in selected_nodes:
            print(f"- {node['prefab_name']} (class: {node['class_name']}, id: {node['id']})")
            for direction, edge in edges_by_node[node['id']]:
                arrow = "→" if direction == 'from' else "←"
                other_id = edge['to_id'] if direction == 'from' else edge['from_id']
                other_class = id_to_class.get(other_id, "UNKNOWN")
                print(f"    {edge['relation_type']} {arrow} {other_id} <{other_class}>")

    return node_ids, selected_nodes, selected_edges

def remove_nodes_by_ids(graph, target_ids: list, verbose: bool = False):
    """
    Remove specific nodes (by ID) and their connected edges.

    Args:
        graph: The scene graph (dict with 'nodes' and 'edges')
        target_ids: List or set of node IDs to remove
        verbose: If True, print summary

    Returns:
        Modified graph (in-place, also returned)
    """
    target_ids = set(target_ids)

    if verbose:
        print(f"🧹 Removing {len(target_ids)} nodes by ID...")

    # Remove nodes
    graph['nodes'] = [n for n in graph['nodes'] if n['id'] not in target_ids]

    # Remove related edges
    graph['edges'] = [
        e for e in graph['edges']
        if e['from_id'] not in target_ids and e['to_id'] not in target_ids
    ]

    return graph

def remove_nodes_by_classes(graph, target_classes: list, verbose: bool = False):
    """
    Remove all nodes whose class_name is in target_classes,
    and remove any edges connected to those nodes.
    Modifies graph in place.
    """
    # Step 1: Identify node ids to remove
    target_ids = {node['id'] for node in graph['nodes'] if node['class_name'] in target_classes}
    return remove_nodes_by_ids(graph, target_ids, verbose=verbose)

def remove_duplicate_prefabs_by_class(graph, target_classes, verbose=False):
    """
    Removes duplicate prefab instances (by prefab_name) for the given target classes.
    Keeps only one node per unique prefab_name.

    Args:
        graph (dict): Scene graph with 'nodes' and 'edges'.
        target_classes (list[str]): Class names to search for duplicates within.
        verbose (bool): If True, prints removed nodes.

    Returns:
        Modified graph (in place).
    """
    _, candidate_nodes, _ = find_nodes_and_edges_by_class(graph, target_classes, verbose=False)

    seen_prefabs = {}
    nodes_to_remove = []

    for node in candidate_nodes:
        prefab = node.get("prefab_name")
        node_id = node.get("id")

        if prefab not in seen_prefabs:
            seen_prefabs[prefab] = node_id  # Keep one
        else:
            nodes_to_remove.append(node_id)

    if verbose and nodes_to_remove:
        print(f"🧹 Removing {len(nodes_to_remove)} duplicate prefab instances:")
        for node in candidate_nodes:
            if node["id"] in nodes_to_remove:
                print(f"  - {node['prefab_name']} (id: {node['id']}, class: {node['class_name']})")

    return remove_nodes_by_ids(graph, nodes_to_remove, verbose=False)
    
def categorize_from_nodes(from_nodes):
    """
    Categorize class_names of from_nodes into 4 buckets based on:
    - GRABBABLE property
    - Whether multiple prefab_names exist for that class
    """
    class_to_prefabs = defaultdict(set)
    class_to_is_grabbable = defaultdict(bool)

    for node in from_nodes:
        cls = node['class_name']
        prefab = node['prefab_name']
        props = node.get('properties') or []

        class_to_prefabs[cls].add(prefab)
        if 'GRABBABLE' in props:
            class_to_is_grabbable[cls] = True

    result = {
        "ambiguous_manipulable_object": [],
        "unambiguous_manipulable_object": [],
        "ambiguous_unmanipulable_object": [],
        "unambiguous_unmanipulable_object": [],
    }

    for cls in class_to_prefabs:
        prefab_count = len(class_to_prefabs[cls])
        is_grabbable = class_to_is_grabbable[cls]

        if is_grabbable:
            if prefab_count > 1:
                result["ambiguous_manipulable_object"].append(cls)
            else:
                result["unambiguous_manipulable_object"].append(cls)
        else:
            if prefab_count > 1:
                result["ambiguous_unmanipulable_object"].append(cls)
            else:
                result["unambiguous_unmanipulable_object"].append(cls)

    return result

def fetch_virtualhome_objects(url:str = "http://virtual-home.org/documentation/master/kb/objects.html"):
    """
    Fetch and parse the VirtualHome object list from the given URL.
    Returns a cleaned pandas DataFrame.
    """
    # Fetch page
    response = requests.get(url)
    soup = BeautifulSoup(response.text, 'html.parser')

    # Find table
    table = soup.find('table')

    # Parse headers
    headers = [header.text.strip() for header in table.find_all('th')]

    # Parse rows
    rows = []
    current_object_name = None

    for row in table.find_all('tr')[1:]:  # skip header
        cells = row.find_all('td')
        cell_texts = [cell.text.strip() for cell in cells]

        if len(cell_texts) == len(headers):
            current_object_name = cell_texts[0]
        else:
            # fill in missing Object Name
            cell_texts = [current_object_name] + cell_texts

        # Clean encoding artifacts
        cleaned_row = [text.replace('â', 'y') for text in cell_texts]
        rows.append(cleaned_row)

    # Create DataFrame
    df = pd.DataFrame(rows, columns=headers)

    # Fill down missing Object Names if any
    df['Object Name'] = df['Object Name'].replace('', pd.NA).ffill()

    return df

def get_ambiguous_manipulable_metadata(
    ambiguous_classes: list = ["book", "dishbowl", "pillow", "clothespile", "towel", "folder"],
    sample: bool = False,
    seed: int = 42
):
    """
    Fetch prefab metadata for ambiguous manipulable objects.
    - Normalize Object Name (lowercase, no underscore).
    - For 'book', only keep Book_XX and PRE_PRO_Book_01/02.
    - If sample=True, sample up to 2 prefabs per class.
    """
    df = fetch_virtualhome_objects()

    normalize = lambda s: s.lower().replace("_", "")
    target_set = set(normalize(cls) for cls in ambiguous_classes)

    mask = df["Object Name"].apply(lambda name: normalize(name) in target_set)
    filtered = df[mask].copy()

    # Special-case filtering for 'book'
    is_book = filtered["Object Name"].apply(lambda s: normalize(s) == "book")
    keep_book = filtered["Prefab Name"].apply(
        lambda name: bool(re.fullmatch(r"Book_\d+", name)) or name in {"PRE_PRO_Book_01", "PRE_PRO_Book_02"}
    )
    filtered = filtered[~is_book | keep_book]

    # Normalize "Object Name" column in output
    filtered["Object Name"] = filtered["Object Name"].apply(normalize)

    # Optional sampling
    if sample:
        random.seed(seed)
        sampled_rows = []
        for cls in ambiguous_classes:
            norm_cls = normalize(cls)
            subset = filtered[filtered["Object Name"] == norm_cls]
            sampled_rows.append(subset.sample(n=min(2, len(subset)), random_state=seed))
        filtered = pd.concat(sampled_rows, ignore_index=True)
        
    # TODO
    manual_prefabs = {
        "dishbowl": ["PRE_PRO_Bowl_01", "FMGP_PRE_Wooden_bowl_1024"],
        "pillow": ["PRE_DEC_Pillow_01_02", "HSHP_PRE_DEC_Pillow_01_01"],
    }
    for cls, new_prefabs in manual_prefabs.items():
        mask = filtered["Object Name"] == cls
        matching_indices = filtered[mask].index

        if len(matching_indices) != len(new_prefabs):
            # print(f"⚠️ Mismatch: {cls} has {len(matching_indices)} rows but {len(new_prefabs)} new prefabs")
            continue

        for idx, new_prefab in zip(matching_indices, new_prefabs):
            filtered.at[idx, "Prefab Name"] = new_prefab

    return filtered

def generate_fixed_waypoint_script(graph, surface_ids):
    """
    Generate a deterministic navigation script for <char0> to walk to and look at surfaces,
    grouped by their containing room, sorted by room and surface IDs.

    Args:
        graph (dict): Scene graph
        surface_ids (list[int]): List of surface node IDs

    Returns:
        List[str]: VirtualHome script lines
    """
    id_to_node = {node["id"]: node for node in graph["nodes"]}
    
    # Group surfaces by their room ID
    room_to_surfaces = {}
    for sid in surface_ids:
        room = find_room_of_node(graph, sid)
        if not room:
            continue  # skip surfaces without room
        room_id = room["id"]
        room_to_surfaces.setdefault(room_id, {"room": room, "surfaces": []})
        room_to_surfaces[room_id]["surfaces"].append(id_to_node[sid])
    
    script = []
    for room_id in sorted(room_to_surfaces):
        room_entry = room_to_surfaces[room_id]
        room = room_entry["room"]
        surfaces = sorted(room_entry["surfaces"], key=lambda n: n["id"])

        script.append(f"<char0> [Walk] <{room['class_name']}> ({room['id']})")
        for surf in surfaces:
            script.append(f"<char0> [Walk] <{surf['class_name']}> ({surf['id']})")
            script.append(f"<char0> [LookAt] <{surf['class_name']}> ({surf['id']})")
            # no ops
            # script.append(f"<char0> [LookAt] <{surf['class_name']}> ({surf['id']})")
    
    return script

def generate_walk_find_script(graph, target_classes):
    """
    Generate script lines like:
        <char0> [Walk] <room> (room_id)      ← added
        <char0> [Walk] <surface> (surface_id)
        <char0> [LookAt] <object> (object_id)
    For all objects in target_classes that are ON surfaces.
    """
    id_to_node = {node['id']: node for node in graph['nodes']}
    script_lines = []

    for edge in graph['edges']:
        if edge['relation_type'] != 'ON':
            continue

        obj_node = id_to_node.get(edge['from_id'])
        surf_node = id_to_node.get(edge['to_id'])

        if not obj_node or not surf_node:
            continue
        if obj_node['class_name'] not in target_classes:
            continue
        
        # Step 1: Walk to room (if any)
        room_node = find_room_of_node(graph, surf_node['id'])
        if room_node:
            script_lines.append(f"<char0> [Walk] <{room_node['class_name']}> ({room_node['id']})")

        # Step 2: Walk to surface
        script_lines.append(f"<char0> [Walk] <{surf_node['class_name']}> ({surf_node['id']})")

        # Step 3: Repeated LookAt
        for _ in range(1):
            script_lines.append(f"<char0> [LookAt] <{obj_node['class_name']}> ({obj_node['id']})")

    return script_lines

def generate_single_object_replacement_script(graph, node, relationships, character:str='<char0>', verbose=False):
    """
    Generates a script for <char1> to pick up an object and place it on a valid surface.
    <char0> walks along but does not interact with objects.

    Args:
        graph: environment graph
        node: a single object node (dict)
        relationships: ON-placement map, from object class to valid surface classes
        verbose: whether to print debug info

    Returns:
        List[str]: list of action lines
    """
    src_room = find_room_of_node(graph, node['id'])
    if not src_room:
        print("❌ Could not find room for object.")
        return None

    dst_surface = choose_valid_surface(graph, node["class_name"], relationships, verbose=verbose)
    if not dst_surface:
        print(f"No valid surface found for object {node['class_name']}")
        return None
    
    dst_room = find_room_of_node(graph, dst_surface['id'])
    if not dst_room:
        print("❌ Could not find room for surface.")
        return None

    script = [
        f'{character} [Walk] <{src_room["class_name"]}> ({src_room["id"]})',
        f'{character} [Grab] <{node["class_name"]}> ({node["id"]})',
        f'{character} [Walk] <{dst_room["class_name"]}> ({dst_room["id"]})',
        f'{character} [Walk] <{dst_surface["class_name"]}> ({dst_surface["id"]})',
        f'{character} [Put] <{node["class_name"]}> ({node["id"]}) <{dst_surface["class_name"]}> ({dst_surface["id"]})'
    ]

    if verbose:
        print("✅ Script generated:")
        for line in script:
            print("  ", line)

    return script

def print_edges_by_class(graph, target_class):
    """
    Print all edges involving nodes of the given class_name,
    grouped by object ID (instance).
    """
    id_to_node = {n['id']: n for n in graph['nodes']}
    
    # Find all nodes of the target class
    target_nodes = [n for n in graph['nodes'] if n['class_name'] == target_class]
    target_ids = set(n['id'] for n in target_nodes)

    # Group edges involving those nodes
    edges_by_id = defaultdict(list)
    for edge in graph['edges']:
        if edge['from_id'] in target_ids or edge['to_id'] in target_ids:
            edges_by_id[edge['from_id'] if edge['from_id'] in target_ids else edge['to_id']].append(edge)

    # Print grouped edges
    for node in target_nodes:
        print(f"\n{node['class_name']} (id: {node['id']}, prefab: {node['prefab_name']})")
        for edge in edges_by_id.get(node['id'], []):
            direction = "→" if edge['from_id'] == node['id'] else "←"
            other_id = edge['to_id'] if direction == "→" else edge['from_id']
            other_node = id_to_node.get(other_id, {'class_name': 'UNKNOWN'})
            print(f"    {edge['relation_type']} {direction} {other_node['class_name']} (id: {other_id})")

def remove_all_objects_on_surfaces(graph, surface_class_names, verbose=False):
    """
    Removes all nodes that are ON any surface whose class_name is in surface_class_names.
    Cleans up all edges involving those nodes.
    """
    # Step 1: Identify surface node IDs
    surface_ids = {node['id'] for node in graph['nodes'] if node['class_name'] in surface_class_names}

    # Step 2: Find ON edges where to_id is a surface
    on_edges = [e for e in graph['edges'] if e['relation_type'] == 'ON' and e['to_id'] in surface_ids]
    object_ids = {e['from_id'] for e in on_edges}

    if verbose and object_ids:
        print(f"🧹 Preparing to remove {len(object_ids)} object nodes from surfaces...")

    # Step 3: Clean up all nodes with those IDs
    graph['nodes'] = [n for n in graph['nodes'] if n['id'] not in object_ids]

    # Step 4: Remove all edges involving those IDs
    graph['edges'] = [
        e for e in graph['edges']
        if e['from_id'] not in object_ids and e['to_id'] not in object_ids
    ]

    if verbose and object_ids:
        print(f"✅ Removed objects: {sorted(object_ids)}")
        
    return graph

def get_node_parents(graph, node_id):
    return [edge['from_id'] for edge in graph['edges'] if edge['to_id'] == node_id]

def get_node_children(graph, node_id):
    return [edge['to_id'] for edge in graph['edges'] if edge['from_id'] == node_id]

def find_room_of_node(graph, node_id):
    """
    Traverses the graph to find the room that contains the given node_id.
    Assumes rooms are top-level containers and reachable via INSIDE or ON edges.
    """
    children = get_node_children(graph, node_id)
    
    # BFS upward until we find a room
    visited = set()
    queue = list(children)
    
    while queue:
        current_id = queue.pop(0)
        current_node = [n for n in graph['nodes'] if n['id'] == current_id][0]
        
        if current_node['class_name'] in ['bathroom', 'kitchen', 'bedroom', 'livingroom', 'diningroom']:
            return current_node  # Found room
        
        if current_id not in visited:
            visited.add(current_id)
            queue.extend(get_node_children(graph, current_id))  # Go upward

    return None  # Room not found

def choose_valid_surface(graph, obj_class, relationships, verbose=False):
    """
    Chooses a valid surface node for a given object class based on relationships.

    Args:
        graph: Environment graph
        obj_class: e.g., "book"
        relationships: dict mapping object class to valid surface classes
        verbose: whether to print fallback info

    Returns:
        A valid surface node (dict), or None if not found.
    """
    valid_surface_classes = relationships.get(obj_class, {}).get("ON", [])
    if not valid_surface_classes:
        if verbose:
            print(f"⚠️ No ON-surfaces defined for '{obj_class}' in relationships.")
        return None

    _, surface_nodes, _ = find_nodes_and_edges_by_class(graph, valid_surface_classes, verbose=verbose)
    if not surface_nodes:
        if verbose:
            print(f"⚠️ No surface nodes in scene for classes: {valid_surface_classes}")
        return None

    return random.choice(surface_nodes)

def filter_nodes_by_prefab(nodes, prefab_names):
    """
    Filters a list of nodes by a given list of prefab names.

    Args:
        nodes (List[dict]): List of node dictionaries.
        prefab_names (List[str]): List of prefab names to keep.

    Returns:
        List[dict]: Filtered list of nodes matching the prefab_names.
    """
    return [node for node in nodes if node.get("prefab_name") in prefab_names]


def filter_nodes_by_class(nodes, class_names):
    """
    Filters a list of nodes by a given list of prefab names.

    Args:
        nodes (List[dict]): List of node dictionaries.
        prefab_names (List[str]): List of prefab names to keep.

    Returns:
        List[dict]: Filtered list of nodes matching the prefab_names.
    """
    return [node for node in nodes if node.get("class_name") in class_names]

def diff_node_edges(graph_old, graph_new, class_name_map=None):
    """
    Prints a node-centric diff of edge changes between two graphs.

    Args:
        graph_old, graph_new: Graphs in VirtualHome format
        class_name_map (optional): Dict mapping node_id to class_name for faster lookup
    """
    from collections import defaultdict

    def edges_by_node(edges):
        edge_map = defaultdict(set)
        for edge in edges:
            key = (edge['relation_type'], edge['to_id'])
            edge_map[edge['from_id']].add(key)
        return edge_map

    # Build class name map
    if class_name_map is None:
        class_name_map = {node['id']: node['class_name'] for node in graph_new['nodes']}
        class_name_map.update({node['id']: node['class_name'] for node in graph_old['nodes']})

    edges_old = edges_by_node(graph_old['edges'])
    edges_new = edges_by_node(graph_new['edges'])

    all_node_ids = set(edges_old.keys()).union(edges_new.keys())
    for node_id in sorted(all_node_ids):
        old = edges_old.get(node_id, set())
        new = edges_new.get(node_id, set())
        added = new - old
        removed = old - new

        if not added and not removed:
            continue

        cls = class_name_map.get(node_id, "unknown")
        print(f"\n📍 {cls} (id: {node_id})")
        for rel, to_id in sorted(added):
            to_cls = class_name_map.get(to_id, "unknown")
            print(f"  ➕ ADDED EDGE: {rel} → {to_cls} (id: {to_id})")
        for rel, to_id in sorted(removed):
            to_cls = class_name_map.get(to_id, "unknown")
            print(f"  ➖ REMOVED EDGE: {rel} → {to_cls} (id: {to_id})")

def replace_prefab_names(graph, target_class: str, new_prefab_names: list, verbose: bool = False):
    """
    Replace the 'prefab_name' of nodes of a given class with new asset names.

    Args:
        graph (dict): The scene graph (modified in-place).
        target_class (str): The class name of nodes to modify (e.g., "book").
        new_prefab_names (list[str]): New prefab names to assign.
        verbose (bool): If True, print before/after info.

    Returns:
        dict: The modified graph.
    """
    count = 0
    for node in graph['nodes']:
        if node['class_name'] == target_class:
            if count < len(new_prefab_names):
                if verbose:
                    print(f"🔁 Changing prefab of node id {node['id']} from '{node['prefab_name']}' to '{new_prefab_names[count]}'")
                node['prefab_name'] = new_prefab_names[count]
                count += 1
            else:
                break  # no more new prefab names

    if verbose and count < len(new_prefab_names):
        print(f"⚠️ Only used {count} out of {len(new_prefab_names)} prefab names (not enough matching nodes).")

    return graph

def get_connected_to_nodes(graph, from_id, relations=["ON", "INSIDE"]):
    """
    Return a list of node dicts that are connected from `from_id` via edges matching specified relation types.

    Args:
        graph (dict): Scene graph containing nodes and edges.
        from_id (int): ID of the source node.
        relations (list[str]): Allowed relation types (default: ["ON", "INSIDE"]).

    Returns:
        list[dict]: List of connected "to" node objects.
    """
    id2node = {node["id"]: node for node in graph["nodes"]}
    to_nodes = []
    for edge in graph["edges"]:
        if edge["from_id"] == from_id and edge["relation_type"] in relations:
            to_node = id2node.get(edge["to_id"])
            if to_node:
                to_nodes.append(to_node)
    return to_nodes

def insert_object_with_placement(graph, prefab_classes, class_placements, target_class, relations, prefab_candidates: list = None, n=1, verbose=False):
    """
    Insert up to n new objects of a given class into the scene graph.
    Each object is placed on a different surface.

    Args:
        graph (dict): Scene graph to modify in place.
        prefab_classes (dict): Mapping from class name to prefab list.
        class_placements (dict): Mapping from class name to placement options.
        target_class (str): The object class to insert (e.g., "book").
        relations (list[str]): Allowed relations (e.g., ["ON", "IN"]).
        n (int): Number of objects to insert.
        verbose (bool): If True, print detailed steps.

    Returns:
        dict: Modified graph
        list[int]: List of inserted node IDs
    """
    inserted_ids = []

    # Step 1: Get prefab candidates
    prefab_candidates = prefab_candidates or prefab_classes.get(target_class, [])
    prefab_candidates = [p for p in prefab_candidates]
    if not prefab_candidates:
        if verbose:
            print(f"❌ No prefab candidates for class '{target_class}'")
        return graph

    # Step 2: Get valid placement surfaces from class_placements
    placement_options = [
        p for p in class_placements.get(target_class, [])
        if p["relation"] in relations
    ]
    if not placement_options:
        if verbose:
            print(f"❌ No placement rules for class '{target_class}' with relations {relations}")
        return graph

    # Step 3: Collect valid surface candidates
    all_surface_candidates = []
    for option in placement_options:
        surface_class = option["destination"]
        rel = option["relation"]
        candidates = [node for node in graph["nodes"] if node["class_name"] == surface_class]
        for node in candidates:
            all_surface_candidates.append((node, rel))

    # Remove surfaces already used within this call
    used_surface_ids = set()

    # Step 4: Limit n to available unique surfaces and prefabs
    available_surfaces = [
        (node, rel) for (node, rel) in all_surface_candidates
        if node["id"] not in used_surface_ids
    ]
    unique_surface_ids = {node["id"] for (node, _) in available_surfaces}
    n_max = min(n, len(unique_surface_ids), len(prefab_candidates))

    if n > n_max:
        if verbose:
            print(f"⚠️ Reducing n from {n} to {n_max} due to limited surfaces or prefabs.")
        n = n_max

    # Step 5: Randomize and place
    random.shuffle(prefab_candidates)
    random.shuffle(available_surfaces)

    prefab_iter = iter(prefab_candidates)
    surface_iter = iter(available_surfaces)

    for _ in range(n):
        try:
            prefab_name = next(prefab_iter)
            while True:
                surface_node, relation_type = next(surface_iter)
                if surface_node["id"] not in used_surface_ids:
                    used_surface_ids.add(surface_node["id"])
                    break
        except StopIteration:
            break  # no more surfaces or prefabs

        new_id = max([node['id'] for node in graph['nodes']], default=0) + 1
        new_node = {
            "id": new_id,
            "prefab_name": prefab_name,
            "class_name": target_class,
            "properties": ["GRABBABLE"],  # optional
        }
        graph['nodes'].append(new_node)
        graph['edges'].append({
            "from_id": new_id,
            "to_id": surface_node["id"],
            "relation_type": relation_type
        })
        # Facing edge (toward room)
        room_node = find_room_of_node(graph, surface_node["id"])
        if room_node:
            graph['edges'].append({
                "from_id": new_id,
                "to_id": room_node["id"],
                "relation_type": "FACING"
            })
            if verbose:
                print(f"   ↪️ Added FACING edge to room '{room_node['class_name']}' (id={room_node['id']})")
        else:
            if verbose:
                print("   ⚠️ Could not find room for FACING edge.")
        
        inserted_ids.append(new_id)

        if verbose:
            print(f"✅ Inserted {target_class} '{prefab_name}' (id={new_id}) {relation_type} {surface_node['class_name']} (id={surface_node['id']})")

    return graph

def extract_minimal_subgraph_by_classes(graph, target_classes: list):
    """
    Extract a minimal subgraph containing all nodes of the given target classes,
    plus any nodes directly connected to them via edges.

    Args:
        graph (dict): Full VirtualHome scene graph with 'nodes' and 'edges'
        target_classes (list[str]): Class names to include as anchors

    Returns:
        dict: Minimal subgraph with 'nodes' and 'edges'
    """
    id_to_node = {node['id']: node for node in graph['nodes']}
    
    # Step 1: Find target nodes
    target_ids = {node['id'] for node in graph['nodes'] if node['class_name'] in target_classes}
    
    # Step 2: Collect all edges involving those nodes
    sub_edges = []
    connected_ids = set(target_ids)  # will expand with linked nodes

    for edge in graph['edges']:
        if edge['from_id'] in target_ids or edge['to_id'] in target_ids:
            sub_edges.append(edge)
            connected_ids.add(edge['from_id'])
            connected_ids.add(edge['to_id'])

    # Step 3: Collect all nodes involved in those edges
    sub_nodes = [id_to_node[nid] for nid in sorted(connected_ids) if nid in id_to_node]

    return {
        "nodes": sub_nodes,
        "edges": sub_edges
    }
    
def find_objects_in_room(graph, room_name: str, target_class: str) -> list[dict]:
    """
    Given a scene graph, a room name, and a target object class,
    return all nodes of that class that are located inside the given room.

    Args:
        graph (dict): Scene graph with 'nodes' and 'edges'.
        room_name (str): Room class name to match (case-insensitive).
        target_class (str): Target object class to search for.

    Returns:
        List[dict]: List of matching object nodes located in the specified room.
    """
    room_name = room_name.lower()
    matching_nodes = []

    for node in graph['nodes']:
        if node['class_name'] != target_class:
            continue

        room_node = find_room_of_node(graph, node['id'])
        if room_node and room_node['class_name'].lower() == room_name:
            matching_nodes.append(node)

    return matching_nodes

def find_objects_on_surfaces_in_room(
    graph: dict,
    room_name: str,
    surface_class: str,
    target_classes: list[str],
    relations: list[str]
) -> list[dict]:
    """
    Find all object nodes in a room that are connected to a surface/container node
    via specific relations (e.g., ON, INSIDE).

    Args:
        graph (dict): Scene graph with 'nodes' and 'edges'.
        room_name (str): Name of the room to search in (e.g., 'bedroom').
        surface_class (str): Class name of the surface/container (e.g., 'wallshelf').
        target_classes (list[str]): List of target object class names (e.g., ['book', 'photoframe']).
        relations (list[str]): List of relation types to include (e.g., ['ON', 'INSIDE']).

    Returns:
        list[dict]: List of matching object nodes.
    """
    # Step 1: Get all surface/container nodes of that class in the room
    surface_nodes = find_objects_in_room(graph, room_name, surface_class)
    surface_ids = {node['id'] for node in surface_nodes}

    # Step 2: Build a map for quick node lookup
    id_to_node = {node['id']: node for node in graph['nodes']}

    # Step 3: Traverse edges to find matching object connections
    matching_objects = []

    for edge in graph['edges']:
        if edge['relation_type'] not in relations:
            continue
        if edge['to_id'] not in surface_ids:
            continue

        obj_node = id_to_node.get(edge['from_id'])
        if obj_node and obj_node['class_name'] in target_classes:
            matching_objects.append(obj_node)

    return matching_objects

def load_prefab_metadata(prefab_path: str = "../resources/PrefabClass.json") -> dict:
    """
    Load prefab metadata from a JSON file.
    
    Args:
        prefab_path (str): Path to the JSON file containing prefab metadata.
        
    Returns:
        dict: Parsed JSON data as a dictionary.
    """
    with open(prefab_path, 'r') as f:
        prefab_data = json.load(f)
        prefab_classes = {}
        classes = []
        for prefab_class in prefab_data["prefabClasses"]:
            cls = prefab_class["className"].replace(" ", "").replace("_", "").lower()
            prefab_classes[cls] = prefab_class["prefabs"]
            classes.append(cls)
    return prefab_classes, classes

def semantic_cls_to_rgb(class_name: str, class_list: list):
    BINS_PER_CHANNEL = 9
    CHANNEL_GAP = 255 // (BINS_PER_CHANNEL - 1)  # 36

    name = class_name.replace(" ", "").replace("_", "").lower()
    try:
        idx = class_list.index(name)
    except ValueError:
        raise ValueError(f"Class '{class_name}' not found.")

    r = ((idx // (BINS_PER_CHANNEL * BINS_PER_CHANNEL)) % BINS_PER_CHANNEL) * CHANNEL_GAP
    g = ((idx // BINS_PER_CHANNEL) % BINS_PER_CHANNEL) * CHANNEL_GAP
    b = (idx % BINS_PER_CHANNEL) * CHANNEL_GAP
    return (r, g, b)

def semantic_rgb_to_cls(rgb: tuple, class_list: list):
    # Reconstruct index → class name list
    BINS_PER_CHANNEL = 9
    CHANNEL_GAP = 255 // (BINS_PER_CHANNEL - 1)  
    
    r, g, b = rgb
    # TODO need to double check this to ensure nothing is off by 1
    r_bin = r // CHANNEL_GAP
    g_bin = g // CHANNEL_GAP
    b_bin = b // CHANNEL_GAP
    # r_bin = round(r / CHANNEL_GAP)
    # g_bin = round(g / CHANNEL_GAP)
    # b_bin = round(b / CHANNEL_GAP)
    idx = r_bin * 81 + g_bin * 9 + b_bin

    if idx >= len(class_list):
        return "unknown"
    return class_list[idx]

def bgr_to_rgb_imgs(imgs):
    return [img[:, :, ::-1].copy() for img in imgs]