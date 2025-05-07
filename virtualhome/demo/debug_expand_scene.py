import sys

sys.path.append('../simulation')
from unity_simulator.comm_unity import UnityCommunication
from utils_demo import add_node, add_edge
import copy

if __name__ == "__main__":
    comm = UnityCommunication(port="8080")
    comm.timeout_wait = 300

    # Step 1: Reset and get the original graph
    comm.reset(4)
    success, graph = comm.environment_graph()
    if not success:
        raise RuntimeError("Failed to get initial environment graph.")

    target_id = 235

    # === Save a copy of the node and its edges ===
    original_node = copy.deepcopy(next((n for n in graph['nodes'] if n['id'] == target_id), None))
    if original_node is None:
        raise ValueError(f"Node with ID {target_id} not found.")

    related_edges = [copy.deepcopy(e) for e in graph['edges'] if e['from_id'] == target_id or e['to_id'] == target_id]

    original_graph = copy.deepcopy(graph)
    success, message = comm.expand_scene(original_graph)
    if not success:
        import pdb; pdb.set_trace()
        print("Attempt1: ❌ Scene expansion failed after restoring node.")

    # === Remove node and its edges ===
    graph['nodes'] = [n for n in graph['nodes'] if n['id'] != target_id]
    graph['edges'] = [e for e in graph['edges'] if e['from_id'] != target_id and e['to_id'] != target_id]
    
    success, message = comm.expand_scene(original_graph)
    if not success:
        import pdb; pdb.set_trace()
        print("Attempt3: ❌ Scene expansion failed after restoring node.")

    success, message = comm.expand_scene(graph)
    if not success:
        import pdb; pdb.set_trace()
        raise RuntimeError("Attempt4: ❌ Scene expansion failed after removing node.")

    # === Restore node and its edges ===
    success, graph = comm.environment_graph()
    
    new_id = max(n['id'] for n in graph['nodes']) + 1  # safe new ID
    original_node['id'] = new_id
    for e in related_edges:
        if e['from_id'] == target_id:
            e['from_id'] = new_id
        if e['to_id'] == target_id:
            e['to_id'] = new_id
    
    graph['nodes'].append(original_node)
    graph['edges'].extend(related_edges)

    # OR: if you want to do it through helper functions (for consistency)
    # add_node(graph, original_node)
    # for edge in related_edges:
    #     add_edge(graph, edge['from_id'], edge['relation_type'], edge['to_id'])
    
    # success, message = comm.expand_scene(original_graph)
    # if not success:
        # import pdb; pdb.set_trace()
        # print("Attempt5: ❌ Scene expansion failed after restoring node.")

    success, message = comm.expand_scene(graph)
    if not success:
        print("Attempt6: ❌ Scene expansion failed after restoring node.")
        import pdb; pdb.set_trace()
        
        success, message = comm.expand_scene(original_graph)
        if not success:
            print("Attempt7: ❌ Scene expansion failed after restoring original graph.")
            import pdb; pdb.set_trace()
            exit(1)
        
    print("✅ Node and edges restored successfully.")
    exit(0)
