import argparse
import os
import json

from graph_utils import *

def parse_args():
    parser = argparse.ArgumentParser(description="Post-process collected data for VirtualHome.")
    parser.add_argument('--data_dir', type=str, default="../../unity_output", help='Directory to save the processed data.')
    parser.add_argument('--datanames', nargs='+', type=str, required=True, help='List of data names to process.')
    return parser.parse_args()

def postprocess_once(data_dir: str, dataname: str):
    simulation_data_dir = os.path.join(data_dir, "0")
    
    pose_path = os.path.join(simulation_data_dir, f"pd_{dataname}.txt")
    if not os.path.isfile(pose_path):
        raise FileNotFoundError(f"Required pose file not found: {pose_path}")
    hip_positions = [] # In unity, hip is the root joint?
    with open(pose_path, "r") as f:
        lines = f.readlines()
        for line in lines[1:]:
            values = line.strip().split()
            if len(values) < 4:
                continue  # skip malformed lines
            x, y, z = map(float, values[1:4])
            hip_positions.append([x, y, z])
    
    gt_data_path = os.path.join(simulation_data_dir, "gt_annotations.json")
    with open(gt_data_path, 'r') as f:
        gt_data = json.load(f)
    if not gt_data:
        print(f"No ground truth data found in {gt_data_path}. Skipping post-processing.")
        return False
    
    frame_data = gt_data.get("frames", {})
    graph = gt_data.get("graph", {})
    
    assert len(hip_positions) == len(frame_data)
    
    entities = []
    for fdata in frame_data:
        frame_id, obs_data = fdata[0], fdata[1]
        unique_ids = set()
        for _, ids in obs_data.items():
            if isinstance(ids, list):
                unique_ids.update(ids)
            else:
                unique_ids.add(ids)
        unique_ids = [int(id) for id in unique_ids]
        nodes = extract_nodes_by_ids(graph["nodes"], unique_ids)
        
        # Filter out nodes with 'GRABBABLE' in their properties
        grabbable_nodes = [node for node in nodes if 'GRABBABLE' in node.get('properties', [])]
        
        class_name_counts = {}
        for node in grabbable_nodes:
            class_name = node.get('class_name', 'Unknown')
            class_name_counts[class_name] = class_name_counts.get(class_name, 0) + 1
        
        # Generate a synthetic caption based on class_name_counts
        if not class_name_counts:
            caption = "I'm not seeing any objects."
        else:
            items = []
            for name, count in class_name_counts.items():
                if count == 1:
                    items.append(f"1 {name}")
                else:
                    items.append(f"{count} {name}s")
                caption = "I'm seeing " + ", ".join(items[:-1])
                if len(items) > 1:
                    caption += f", and {items[-1]}."
                else:
                    caption += f"{items[-1]}."
        
        entity = {
            "time": frame_id * 0.2,  # Assuming each frame is 0.25 seconds apart
            "waypoint": -1,
            "base_position": hip_positions[frame_id],
            "base_caption": caption,
            "wrist_position": hip_positions[frame_id],
            "wrist_caption": "",
            "start_frame": frame_id,
            "end_frame": frame_id+1,
        }
        entities.append(entity)
        
    out_path = os.path.join(simulation_data_dir, "caption_synthetic.json")
    with open(out_path, "w") as f:
        json.dump(entities, f, indent=2)
    print(f"✅ Saved captions to {out_path}")
        

def run(args):
    for dataname in args.datanames:
        data_dir = os.path.join(args.data_dir, dataname)
        if not os.path.exists(data_dir):
            print(f"Directory does not exist: {data_dir}")
            continue
        postprocess_once(data_dir, dataname)

if __name__ == "__main__":
    args = parse_args()
    run(args)