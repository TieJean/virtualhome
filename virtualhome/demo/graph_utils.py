from collections import defaultdict
import requests
from bs4 import BeautifulSoup
import pandas as pd
import re
import random
from utils_demo import *

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

def load_relationships(filepath: str):
    """
    Load lines like 'book ON bookshelf' into:
    {
        'book': {
            'ON': [surface1, surface2, ...]
        },
        ...
    }
    """
    result = {}

    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue  # skip empty lines if any
            try:
                obj, relation, surface = line.split()
            except ValueError:
                raise ValueError(f"Line not formatted as '<object> ON <surface>': {line}")
            
            if obj not in result:
                result[obj] = {}
            if relation not in result[obj]:
                result[obj][relation] = []
            result[obj][relation].append(surface)

    return result

def no_ops(char="<char0>", count=3):
    """
    Return a list of no-op [Stand] actions to simulate idle time.
    """
    # TODO: may need to use [Sit] or [Lie] in some cases
    return [f"{char} [StandUp]"] * count

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
        
        print("here")

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