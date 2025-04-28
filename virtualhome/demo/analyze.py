import pandas as pd
from collections import Counter

import sys

sys.path.append('../simulation')
from unity_simulator.comm_unity import UnityCommunication

def generate_scene_graph_pandas(graph):
    nodes = graph['nodes']
    edges = graph['edges']

    # ---- Node analysis ----
    filtered_nodes = [node for node in nodes if node.get('category') != 'Rooms']
    total_nodes = len(filtered_nodes)

    # Create dataframe
    node_df = pd.DataFrame(filtered_nodes)
    node_summary = (
        node_df['class_name']
        .value_counts()
        .rename_axis('class_name')
        .reset_index(name='count')
    )
    node_summary['percentage'] = (node_summary['count'] / total_nodes) * 100

    print("\n=== Object Class Analysis ===")
    print(node_summary)

    # ---- Edge analysis ----
    edge_df = pd.DataFrame(edges)
    total_edges = len(edge_df)

    edge_summary = (
        edge_df['relation_type']
        .value_counts()
        .rename_axis('relation_type')
        .reset_index(name='count')
    )
    edge_summary['percentage'] = (edge_summary['count'] / total_edges) * 100

    print("\n=== Edge Relation Analysis ===")
    print(edge_summary)

    return node_df, edge_df, node_summary, edge_summary

def analyze_on_inside_relationships(edge_df, node_df):
    # Create ID -> class_name mapping
    id_to_class = node_df.set_index('id')['class_name'].to_dict()

    # Filter edges of interest
    interesting_edges = edge_df[edge_df['relation_type'].isin(['INSIDE', 'ON'])]

    # Map and skip Unknowns
    mapped = []
    for _, row in interesting_edges.iterrows():
        from_obj = id_to_class.get(row['from_id'])
        to_obj = id_to_class.get(row['to_id'])
        if from_obj is None or to_obj is None:
            continue  # skip unknowns
        mapped.append((relation_type := row['relation_type'], from_obj, to_obj))

    # Group and count
    counter = Counter(mapped)

    # Print nicely
    print("\n=== INSIDE / ON Relation Triples (Sorted by Frequency) ===")
    total = sum(counter.values())
    for (relation_type, from_obj, to_obj), count in counter.most_common():
        percentage = (count / total) * 100
        print(f"{from_obj:20s} {relation_type:8s} {to_obj:20s} {count:5d} ({percentage:.2f}%)")

    return counter

def analyze_on_inside_relationships_instance_level(edge_df, node_df):
    # Create ID -> instance label mapping
    id_to_instance_name = node_df.apply(lambda row: f"{row['class_name']}_{row['id']}", axis=1)
    id_to_instance_name = pd.Series(id_to_instance_name.values, index=node_df['id']).to_dict()

    # Filter edges of interest
    interesting_edges = edge_df[edge_df['relation_type'].isin(['INSIDE', 'ON'])]

    # Map and skip Unknowns
    mapped = []
    for _, row in interesting_edges.iterrows():
        from_obj = id_to_instance_name.get(row['from_id'])
        to_obj = id_to_instance_name.get(row['to_id'])
        if from_obj is None or to_obj is None:
            continue  # skip unknowns
        mapped.append((relation_type := row['relation_type'], from_obj, to_obj))

    # Group and count
    counter = Counter(mapped)

    # Print nicely
    print("\n=== Instance-Level INSIDE / ON Relation Triples (Sorted by Frequency) ===")
    total = sum(counter.values())
    for (relation_type, from_obj, to_obj), count in counter.most_common():
        percentage = (count / total) * 100
        print(f"{from_obj:25s} {relation_type:8s} {to_obj:25s} {count:5d} ({percentage:.2f}%)")

    return counter

def analyze_prefabs_by_class(node_df, show_prefab_list=False):
    """
    Group prefab names by class_name and output a clean table:
    - class_name
    - instance_count
    - prefab_count
    - optionally: prefab_list
    Also return lists of unambiguous and ambiguous classes.
    """
    # Create prefab list per class
    grouped = (
        node_df
        .groupby('class_name')['prefab_name']
        .apply(list)
        .reset_index()
    )

    # Process into a clean table
    grouped['instance_count'] = grouped['prefab_name'].apply(len)
    grouped['distinct_prefabs'] = grouped['prefab_name'].apply(lambda prefabs: list(set(prefabs)))
    grouped['prefab_count'] = grouped['distinct_prefabs'].apply(len)
    grouped['prefab_list'] = grouped['distinct_prefabs'].apply(lambda x: ", ".join(sorted(x)))

    # Select columns
    if show_prefab_list:
        table = grouped[['class_name', 'instance_count', 'prefab_count', 'prefab_list']]
    else:
        table = grouped[['class_name', 'instance_count', 'prefab_count']]

    # Sort: first by instance_count (desc), then prefab_count (desc)
    table = table.sort_values(by=['instance_count', 'prefab_count'], ascending=[False, False])

    # Find ambiguous and unambiguous classes
    ambiguous_classes = table[table['prefab_count'] > 1]['class_name'].tolist()
    unambiguous_classes = table[table['prefab_count'] == 1]['class_name'].tolist()

    # Print table
    print("\n=== Prefab Table Grouped by Class (Sorted) ===")
    print(table.to_string(index=False))

    # Print ambiguous / unambiguous classes
    print("\n=== Unambiguous Classes (Single Prefab) ===")
    print(unambiguous_classes)

    print("\n=== Ambiguous Classes (Multiple Prefabs) ===")
    print(ambiguous_classes)

    return table, unambiguous_classes, ambiguous_classes

if __name__ == "__main__":
    comm = UnityCommunication(port="8080")
    comm.timeout_wait = 300 
    success, graph = comm.environment_graph()
    if success:
        node_df, edge_df, node_summary, edge_summary = generate_scene_graph_pandas(graph)
    else:
        print("Failed to get environment graph.")
        exit(1)
        
    analyze_on_inside_relationships(edge_df, node_df)
    analyze_prefabs_by_class(node_df)
    # analyze_on_inside_relationships_instance_level(edge_df, node_df)
