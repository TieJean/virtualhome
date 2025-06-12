import argparse
import os
import cv2
import numpy as np
import imageio.v3 as iio
import json
import tqdm

def parse_args():
    parser = argparse.ArgumentParser(description="Extract classes from a graph JSON file.")
    parser.add_argument("--data_dir", type=str, required=True, help="Path to the directory that contains all image files.")
    parser.add_argument("--depth_thresh", type=float, default=5.0, help="Depth threshold for filtering objects.")
    return parser.parse_args()

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

def extract_classes(args):
    seg_class_files = sorted([
        f for f in os.listdir(args.data_dir)
        if f.endswith('_seg_class.png')
    ])
    depth_files = sorted([
        f for f in os.listdir(args.data_dir)
        if f.endswith('_depth.exr')
    ])
    assert len(seg_class_files) == len(depth_files), "Mismatch between number of seg_class and depth files"
    
    detected_classes = []
    for seg_cls_filename, depth_filename in tqdm.tqdm(zip(seg_class_files, depth_files), total=len(seg_class_files), desc="Extracting classes"):
        # depth
        depth_path = os.path.join(args.data_dir, depth_filename)
        depth_img = iio.imread(depth_path)
        assert depth_img.dtype in (np.float32, np.float64), f"Unexpected dtype: {depth_img.dtype}"
        if depth_img.ndim == 3 and depth_img.shape[2] == 4:
            depth_scalar_img = depth_img[..., 0]
        else:
            depth_scalar_img = depth_img
            
        # seg_cls
        seg_cls_path = os.path.join(args.data_dir, seg_cls_filename)
        if not os.path.isfile(seg_cls_path):
            print(f"File not found: {seg_cls_path}")
            continue
        
        bgr_img = cv2.imread(seg_cls_path)  # BGR format
        if bgr_img is None:
            raise ValueError(f"Failed to read image: {seg_cls_path}")
        # Convert to RGB
        rgb_img = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2RGB)
        
        valid_mask = (depth_scalar_img < 3.0)
        # Apply depth mask to RGB segmentation image
        rgb_masked = rgb_img[valid_mask]
        # Remove black pixels and get unique colors
        non_black = rgb_masked[~np.all(rgb_masked == 0, axis=1)]
        unique_colors = np.unique(non_black, axis=0)

        for color in unique_colors:
            class_name = semantic_rgb_to_cls(color, args.class_list)
            detected_classes.append(class_name)

    return list(set(detected_classes))

if __name__ == "__main__":
    args = parse_args()
    prefab_classes, class_list = load_prefab_metadata("../resources/PrefabClass.json")
    
    args.class_list = class_list
    classes = extract_classes(args)
    print("Extracted classes:", classes)
    