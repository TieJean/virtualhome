from collections import defaultdict
import requests
from bs4 import BeautifulSoup
import pandas as pd
import re
import random

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

def remove_nodes_by_class(graph, target_classes: list, verbose: bool = False):
    """
    Remove all nodes whose class_name is in target_classes,
    and remove any edges connected to those nodes.
    Modifies graph in place.
    """
    # Step 1: Identify node ids to remove
    target_ids = {node['id'] for node in graph['nodes'] if node['class_name'] in target_classes}
    
    if verbose:
        print(f"\n=== Removing {len(target_ids)} ambiguous nodes and their edges ===")

    # Step 2: Filter nodes
    original_node_count = len(graph['nodes'])
    graph['nodes'] = [node for node in graph['nodes'] if node['id'] not in target_ids]
    
    # Step 3: Filter edges
    original_edge_count = len(graph['edges'])
    graph['edges'] = [
        edge for edge in graph['edges']
        if edge['from_id'] not in target_ids and edge['to_id'] not in target_ids
    ]

    if verbose:
        removed_nodes = original_node_count - len(graph['nodes'])
        removed_edges = original_edge_count - len(graph['edges'])
        print(f"Removed {removed_nodes} nodes and {removed_edges} edges.")

    return graph  # (optional, since it's in-place)


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
    ambiguous_classes: list = ["book", "mug", "plate", "dishbowl", "pillow", "clothespile", "towel", "folder"],
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
    return [f"{char} [Stand]"] * count

def generate_walk_find_script(graph, target_classes):
    """
    Generate script lines like:
        <char0> [Walk] <surface> (surface_id)
        <char0> [Find] <object> (object_id)
    For all objects in target_classes (default: ambiguous_manipulable_objects)
    """
    id_to_node = {node['id']: node for node in graph['nodes']}

    # Find all ON edges where from_node is in target_classes
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

        script_lines.append(f"<char0> [Walk] <{surf_node['class_name']}> ({surf_node['id']})")
        script_lines.append(f"<char0> [Find] <{obj_node['class_name']}> ({obj_node['id']})")
        script_lines.extend(no_ops())

    return script_lines

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
