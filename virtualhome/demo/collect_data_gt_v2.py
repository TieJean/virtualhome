from tqdm import tqdm
import os, json, argparse
import sys
import cv2
import imageio.v3 as iio

sys.path.append('../simulation')
from unity_simulator.comm_unity import UnityCommunication
from utils_demo import *
from graph_utils import *

def parse_args():
    parser = argparse.ArgumentParser(description="Post-process collected data for VirtualHome.")
    parser.add_argument('--data_dir', type=str, default="../../unity_output", help='Directory to save the processed data.')
    parser.add_argument('--datanames', nargs='+', type=str, required=True, help='List of data names to process.')
    return parser.parse_args()

def postprocess_visibility(data_dir: str, dataname: str, class_list: dict):
    simulation_data_dir = os.path.join(data_dir, "0")
    
    image_dir = os.path.join(data_dir, "images")
    os.makedirs(image_dir, exist_ok=True)
    for root, _, files in os.walk(image_dir):
        for file in files:
            os.remove(os.path.join(root, file))
            
     # Load pose file
    pose_path = os.path.join(simulation_data_dir, f"pd_{dataname}.txt")
    if not os.path.isfile(pose_path):
        raise FileNotFoundError(f"Missing pose file: {pose_path}")
    
    positions = []
    with open(pose_path, "r") as f:
        lines = f.readlines()
        for line in lines[1:]:
            values = line.strip().split()
            if len(values) < 4:
                continue
            x1, y1, z1 = map(float, values[1+5*3:4+5*3])
            x2, y2, z2 = map(float, values[1+6*3:4+6*3])
            positions.append([(x1+x2)/2, (y1+y2)/2, ((z1+z2)/2)])
            
    # Load segmentation files
    seg_inst_files = sorted([
        f for f in os.listdir(simulation_data_dir)
        if f.endswith('_seg_inst.png')
    ])
    seg_class_files = sorted([
        f for f in os.listdir(simulation_data_dir)
        if f.endswith('_seg_class.png')
    ])
    normal_files = sorted([
        f for f in os.listdir(simulation_data_dir)
        if f.endswith('_normal.png')
    ])
    assert len(seg_class_files) == len(positions), (
        f"Mismatch: {len(seg_class_files)} seg_class files vs {len(positions)} hip positions"
    )
    assert len(seg_class_files) == len(normal_files), (
        f"Mismatch: {len(seg_class_files)} seg_class files vs {len(normal_files)} normal files"
    )
    assert len(seg_class_files) == len(seg_inst_files), (
        f"Mismatch: {len(seg_class_files)} seg_class files vs {len(seg_inst_files)} seg_inst files"
    )
    
    instance_color_file = os.path.join(simulation_data_dir, "instance_colors.json")
    with open(instance_color_file, "r") as f:
        instance_colors = json.load(f)
    if instance_colors is None:
        raise ValueError(f"Instance colors file is empty: {instance_color_file}")
    
    with open(os.path.join(simulation_data_dir, "agent_graph.json"), "r") as f:
        agent_graph = json.load(f)
    if agent_graph is None:
        raise ValueError(f"Agent graph file is empty: {os.path.join(simulation_data_dir, 'agent_graph.json')}")
    
    all_frame_data = []
    annotated_imgs = []
    total_frames = len(seg_inst_files)
    for seg_inst_filename, seg_class_filename, normal_filename, position in tqdm(zip(seg_inst_files, seg_class_files, normal_files, positions), desc=f"Processing frames for {dataname}", total=total_frames):
        idx = normal_files.index(normal_filename)
        
        seg_inst_path = os.path.join(simulation_data_dir, seg_inst_filename)
        seg_class_path = os.path.join(simulation_data_dir, seg_class_filename)
        normal_path = os.path.join(simulation_data_dir, normal_filename)
        seg_inst = cv2.imread(seg_inst_path)  # shape: H, W, 3 (BGR)
        if seg_inst is None:
            raise ValueError(f"Failed to read seg_inst file: {seg_inst_path}")
        seg_class = cv2.imread(seg_class_path)  # shape: H, W, 3 (BGR)
        if seg_class is None:
            raise ValueError(f"Failed to read seg_class file: {seg_class_path}")
        normal_img = cv2.imread(normal_path)  # shape: H, W, 3 (BGR)
        if normal_img is None:
            raise ValueError(f"Failed to read normal file: {normal_path}")
        annotated_img = normal_img.copy()
        
        output_filename = os.path.join(image_dir, f"{idx:06d}.png")
        cv2.imwrite(output_filename, normal_img)
        
        frame_nodes = []
        unique_inst_colors = np.unique(seg_inst.reshape(-1, 3), axis=0)
        for inst_color in unique_inst_colors:
            if np.all(inst_color == 0):
                continue  # skip background
            
            # Mask for this instance color
            mask_inst = np.all(seg_inst == inst_color, axis=-1)

            # Only process if enough pixels (avoid noise)
            if np.sum(mask_inst) < 10:
                continue
            
            # For those pixels, get most common class color
            class_colors, counts = np.unique(seg_class[mask_inst].reshape(-1, 3), axis=0, return_counts=True)
            class_color = class_colors[np.argmax(counts)]
            
            # Find matching node in the graph
            matched_node = None
            for node in agent_graph["nodes"]:
                node_id = str(node["id"])
                prefab_name = node.get("prefab_name", "")
                rgb_f = instance_colors.get(node_id)
                if rgb_f is None:
                    continue
                # Convert to BGR uint8 for OpenCV
                node_inst_color = np.array([rgb_f[2], rgb_f[1], rgb_f[0]]) * 255
                node_inst_color = node_inst_color.astype(np.uint8)
                # Check instance color match (with tolerance)
                if not np.allclose(inst_color, node_inst_color, atol=2):
                    continue
                # Get class color for this node
                # If you have a class_name_to_bgr/class_name_to_color function, use that
                # Here, we assume semantic_cls_to_bgr exists
                node_class_color = semantic_cls_to_bgr(node["class_name"], class_list)
                if not np.allclose(class_color, node_class_color, atol=2):
                    continue
                matched_node = node
                break

            if matched_node is not None:
                frame_nodes.append(matched_node)

                # --- Draw bounding box for this instance ---
                mask_final = (mask_inst & np.all(seg_class == class_color, axis=-1)).astype(np.uint8) * 255
                contours, _ = cv2.findContours(mask_final, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                for cnt in contours:
                    x, y, w, h = cv2.boundingRect(cnt)
                    if w < 4 or h < 4:
                        continue
                    # Instance color as BGR tuple
                    color = tuple(int(v) for v in inst_color)
                    cv2.rectangle(annotated_img, (x, y), (x + w, y + h), color, 2)
                    cv2.putText(
                        annotated_img,
                        matched_node.get("prefab_name", ""),
                        (x, y - 6),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        color,
                        1,
                        cv2.LINE_AA,
                    )
            
        # cv2.imwrite(os.path.join("../../outputs/debug.png"), annotated_img)
        # annotated_imgs.append(annotated_img)
        frame_data = {
            "frame_idx": idx,
            "frame_nodes": frame_nodes,
        }
        all_frame_data.append(frame_data)

    if annotated_imgs:
        out_video_path = os.path.join(data_dir, f"{dataname}_instances_annotated.mp4")
        height, width = annotated_imgs[0].shape[:2]
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        fps = 5  # Or adjust as appropriate
        out = cv2.VideoWriter(out_video_path, fourcc, fps, (width, height))
        for img in annotated_imgs:
            # Ensure all images are 3 channels
            if img.shape[2] == 4:
                img = img[:, :, :3]
            out.write(img)
        out.release()
        print(f"[Info] Wrote annotated instance video to {out_video_path}")
    else:
        print("[Warning] No annotated images to save.")
        
    gt_path = os.path.join(simulation_data_dir, "gt_annotations.json")
    with open(gt_path, "w") as f:
        json.dump(all_frame_data, f, indent=2)
    print(f"Saved gt annotations to {gt_path}")
    

def run(args):
    _, class_list = load_prefab_metadata("../resources/PrefabClass.json")
    
    for dataname in args.datanames:
        data_dir = os.path.join(args.data_dir, dataname)
        postprocess_visibility(data_dir, dataname, class_list)

if __name__ == "__main__":
    args = parse_args()
    run(args)