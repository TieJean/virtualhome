import argparse
import json
import os
import cv2
import numpy as np

def parse_args():
    parser = argparse.ArgumentParser(description="Debug instance colors for a given class in a single seg_inst image.")
    parser.add_argument("--seg_inst_path", type=str, required=True, help="Path to seg_inst.png")
    parser.add_argument("--instance_colors_path", type=str, required=True, help="Path to instance_colors.json")
    parser.add_argument("--graph_path", type=str, required=True, help="Path to agent_graph.json or graph.json")
    parser.add_argument("--class_name", type=str, required=True, help="Target class name (e.g., 'book')")
    return parser.parse_args()

def main():
    args = parse_args()
    
    # Load inputs
    seg_inst = cv2.imread(args.seg_inst_path, cv2.IMREAD_UNCHANGED)  # May have alpha
    if seg_inst is None:
        raise FileNotFoundError(f"Cannot read image: {args.seg_inst_path}")
    if seg_inst.shape[2] == 4:
        seg_inst = seg_inst[:, :, :3]

    with open(args.instance_colors_path, "r") as f:
        instance_colors = json.load(f)

    with open(args.graph_path, "r") as f:
        graph = json.load(f)

    # Normalize target class
    target_class = args.class_name.strip().lower()
    matching_nodes = [
        node for node in graph["nodes"]
        if node.get("class_name", "").strip().lower() == target_class
    ]

    if not matching_nodes:
        print(f"No matching nodes for class: {target_class}")
        return

    matching_colors = []
    print(f"Found {len(matching_nodes)} instances of class '{target_class}':")
    for node in matching_nodes:
        uid = str(node["id"])
        class_name = node.get("class_name", "?")
        color = instance_colors.get(uid)
        if color:
            matching_colors.append((uid, color))
            print(f" - ID: {uid} | class: {class_name} → RGB (float): {color}")
        else:
            print(f" - ID: {uid} | class: {class_name} has no color entry.")

    # Optional: Show a visualization of those regions
    mask = np.zeros(seg_inst.shape[:2], dtype=np.uint8)
    for _, rgb_f in matching_colors:
        bgr_uint8 = (
            int(round(rgb_f[2] * 255)),
            int(round(rgb_f[1] * 255)),
            int(round(rgb_f[0] * 255)),
        )
        m = cv2.inRange(seg_inst, np.array(bgr_uint8), np.array(bgr_uint8))
        mask = cv2.bitwise_or(mask, m)

    overlay = seg_inst.copy()
    overlay[mask > 0] = [0, 0, 255]  # highlight matching in red

    # Draw bounding boxes for each instance
    for node in matching_nodes:
        uid = str(node["id"])
        class_name = node.get("class_name", "?")
        rgb_f = instance_colors.get(uid)
        if not rgb_f:
            continue
        bgr_uint8 = (
            int(round(rgb_f[2] * 255)),
            int(round(rgb_f[1] * 255)),
            int(round(rgb_f[0] * 255)),
        )
        instance_mask = cv2.inRange(seg_inst, np.array(bgr_uint8), np.array(bgr_uint8))
        contours, _ = cv2.findContours(instance_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            x, y, w, h = cv2.boundingRect(cnt)
            if w < 4 or h < 4:
                continue  # skip tiny regions
            label = f"ID: {uid} | {class_name}"
            print(f"Bounding box for node: {label} at (x={x}, y={y}, w={w}, h={h})")
            cv2.rectangle(overlay, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(
                overlay,
                label,
                (x, y - 6),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                1,
                cv2.LINE_AA,
            )

    # For every node with a bounding box, check if any other node is mapped to the same color
    checked_uids = set()
    for node in matching_nodes:
        uid = str(node["id"])
        if uid in checked_uids:
            continue
        rgb_f = instance_colors.get(uid)
        if not rgb_f:
            continue
        bgr_uint8 = [
            int(round(rgb_f[2] * 255)),
            int(round(rgb_f[1] * 255)),
            int(round(rgb_f[0] * 255)),
        ]
        # Check if this node actually has a bounding box in the image
        instance_mask = cv2.inRange(seg_inst, np.array(bgr_uint8), np.array(bgr_uint8))
        contours, _ = cv2.findContours(instance_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            continue
        checked_uids.add(uid)
        same_color_nodes = []
        for other_uid, other_rgb_f in instance_colors.items():
            if other_uid == uid:
                continue
            other_bgr_uint8 = [
                int(round(other_rgb_f[2] * 255)),
                int(round(other_rgb_f[1] * 255)),
                int(round(other_rgb_f[0] * 255)),
            ]
            if other_bgr_uint8 == bgr_uint8:
                same_color_nodes.append((other_uid, other_rgb_f, other_bgr_uint8))
        if same_color_nodes:
            print(f"Nodes mapped to the same color as node {uid} (BGR {bgr_uint8}):")
            for other_uid, other_rgb_f, other_bgr_uint8 in same_color_nodes:
                print(f" - ID: {other_uid} → RGB (float): {other_rgb_f}, BGR (uint8): {other_bgr_uint8}")
        else:
            print(f"No node in instance_colors is mapped to the same color as node {uid} (BGR {bgr_uint8}).")

    # Save the overlay image instead of displaying it
    output_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../outputs'))
    os.makedirs(output_dir, exist_ok=True)
    output_filename = f"matched_instances_{target_class}.png"
    output_path = os.path.join(output_dir, output_filename)
    cv2.imwrite(output_path, overlay)
    print(f"Saved overlay image to {output_path}")

if __name__ == "__main__":
    main()
