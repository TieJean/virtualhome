import argparse
import os
import subprocess
from glob import glob
from tqdm import tqdm
import cv2    
import tempfile, shutil  
import json
import numpy as np
import sys

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from graph_utils import *

def parse_args():
    parser = argparse.ArgumentParser(description="Convert Unity output PNGs to MP4 videos using ffmpeg.")
    parser.add_argument(
        "--unity_output_dir", 
        type=str, 
        default='../../unity_output/',
        help="Path to unity_output directory"
    )
    parser.add_argument(
        "--datanames", 
        type=str, 
        nargs='+', 
        required=True, 
        help="List of dataname folders to process"
    )
    parser.add_argument(
        "--make_stitched_video",
        action='store_true',
        help="If set, will create a stitched video of all bbox videos."
    )
    return parser.parse_args()

def collect_sorted_images(folder, suffix):
    return sorted(glob(os.path.join(folder, f'*{suffix}.png')))

def make_bbox_video(normal_paths, inst_paths, instance_colors, graph, cls_paths, class_list, out_path, fps=5):
    
    def _draw_bounding_box(image, instance_mask, class_mask, instance_colors, class_colors):
        # Modern Palette (Pastels/Neon)
        CLASS_PALETTE = {
            "book":     (  0,   0, 255), "folder":   (  0, 255,   0),
            "toy":      (255,   0,   0), "magazine": (  0, 255, 255),
            "cereal":   (255, 128, 0),   "bananas":  (255, 255,   0),
            "cupcake":  (255, 192, 203), "mincedmeat": (128, 0, 128),
            "apple":    (0, 165, 255),   "creamybuns": (0, 255, 255),
        }
        TARGET_CLASSES = set(CLASS_PALETTE.keys())
        
        # Ensure 3-channel
        if instance_mask.ndim == 2: instance_mask = cv2.cvtColor(instance_mask, cv2.COLOR_GRAY2BGR)
        elif instance_mask.shape[2] == 4: instance_mask = instance_mask[:, :, :3]
        if class_mask.ndim == 2: class_mask = cv2.cvtColor(class_mask, cv2.COLOR_GRAY2BGR)
        elif class_mask.shape[2] == 4: class_mask = class_mask[:, :, :3]

        out = image.copy()

        for node in graph["nodes"]:
            cls_name = node.get("class_name", "").lower()
            if cls_name not in TARGET_CLASSES: continue

            uid = str(node["id"])
            rgb_f = instance_colors.get(uid)
            if rgb_f is None: continue

            bgr_uint8 = (int(round(rgb_f[2]*255)), int(round(rgb_f[1]*255)), int(round(rgb_f[0]*255)))
            
            # Masking
            mask_instance = cv2.inRange(instance_mask, np.array(bgr_uint8), np.array(bgr_uint8))
            bgr_class = class_colors.get(cls_name, (0,0,0))
            mask_class = cv2.inRange(class_mask, np.array(bgr_class), np.array(bgr_class))
            mask = cv2.bitwise_and(mask_instance, mask_class)
            
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not contours: continue

            color = CLASS_PALETTE.get(cls_name, (255, 255, 255))
            
            # Draw contours
            for cnt in contours:
                x, y, w, h = cv2.boundingRect(cnt)
                if w < 4 or h < 4: continue
                
                # Thinner, cleaner line
                cv2.rectangle(out, (x, y), (x + w, y + h), color, 2)
                
                # Label with background for readability
                label = cls_name
                (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                
                # Text Background
                cv2.rectangle(out, (x, y - 20), (x + tw + 4, y), color, -1)
                # Text (White or Black depending on contrast? Using White/Black fixed for now)
                cv2.putText(out, label, (x + 2, y - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1, cv2.LINE_AA)

        return out

    if not normal_paths or not inst_paths or not cls_paths: return

    class_colors = {}
    target_list = ["book", "folder", "toy", "magazine", "bananas", "cupcake", "cereal", "mincedmeat", "apple", "creamybuns"]
    for cls_name in target_list:
        class_colors[cls_name] = semantic_cls_to_bgr(cls_name, class_list)

    tmp_dir = tempfile.mkdtemp()
    try:
        # Use tqdm here for individual video generation progress
        for idx, (n_path, i_path, c_path) in enumerate(zip(normal_paths, inst_paths, cls_paths)):
            img_normal = cv2.imread(n_path)
            img_inst   = cv2.imread(i_path, cv2.IMREAD_UNCHANGED)
            img_cls    = cv2.imread(c_path, cv2.IMREAD_UNCHANGED)
            drawn      = _draw_bounding_box(img_normal, img_inst, img_cls, instance_colors, class_colors)
            cv2.imwrite(os.path.join(tmp_dir, f"frame_{idx:04d}_bbox.png"), drawn)

        annotated_frames = sorted(glob(os.path.join(tmp_dir, "frame_*.png")))
        make_video_ffmpeg(annotated_frames, out_path, fps=fps)
    finally:
        shutil.rmtree(tmp_dir, ignore_errors=True)

def make_video_ffmpeg(image_paths, out_path, fps=5):
    if not image_paths: return
    
    # Filename parsing logic
    folder = os.path.dirname(image_paths[0])
    sample_name = os.path.basename(image_paths[0])
    try:
        prefix, _, remainder = sample_name.split('_', 2) 
    except ValueError:
        return
    suffix = remainder.rsplit('.', 1)[0]
    pattern = os.path.join(folder, f"{prefix}_%04d_{suffix}.png")

    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    command = [
        'ffmpeg', '-y', '-framerate', str(fps), '-i', pattern,
        '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-r', str(fps), out_path
    ]
    try:
        subprocess.run(command, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except subprocess.CalledProcessError:
        print(f"[Error] ffmpeg failed for {out_path}")

def stitch_bbox_videos(video_paths, output_path, max_cols=4, target_width=480):
    """
    Creates a 'Gallery View' stitched video.
    - Resizes all videos to 'target_width'.
    - Adds padding and a modern dark background.
    """
    
    # 1. Initialize Captures
    caps = [cv2.VideoCapture(p) for p in video_paths]
    if not caps: return

    # 2. Get Input Specs (assume first video represents aspect ratio)
    orig_w = caps[0].get(cv2.CAP_PROP_FRAME_WIDTH)
    orig_h = caps[0].get(cv2.CAP_PROP_FRAME_HEIGHT)
    fps = caps[0].get(cv2.CAP_PROP_FPS)
    
    # 3. Calculate Gallery Dimensions
    scale = target_width / orig_w
    cell_w = int(target_width)
    cell_h = int(orig_h * scale)
    
    # Style constants
    PADDING = 20
    LABEL_H = 0  # No text labels
    BG_COLOR = (30, 30, 30) # Soft Dark Grey (Modern VS Code style)
    
    n_videos = len(caps)
    cols = min(n_videos, max_cols)
    import math
    rows = math.ceil(n_videos / cols)
    
    # Final Canvas Size
    grid_w = (cols * cell_w) + ((cols + 1) * PADDING)
    grid_h = (rows * (cell_h + LABEL_H)) + ((rows + 1) * PADDING)
    
    # 4. Prepare Writer
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out_writer = cv2.VideoWriter(output_path, fourcc, fps, (grid_w, grid_h))

    # Determine max frames
    frame_counts = [int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) for cap in caps]
    max_frames = max(frame_counts)

    print(f"Stitching {n_videos} videos into {grid_w}x{grid_h} layout...")

    for frame_idx in tqdm(range(max_frames), desc="Stitching Frames"):
        # Create blank canvas
        canvas = np.full((grid_h, grid_w, 3), BG_COLOR, dtype=np.uint8)
        
        for i, cap in enumerate(caps):
            # Grid position
            c = i % cols
            r = i // cols
            
            x_offset = PADDING + (c * (cell_w + PADDING))
            y_offset = PADDING + (r * (cell_h + LABEL_H + PADDING))
            
            # Read Frame
            if frame_idx < frame_counts[i]:
                ret, frame = cap.read()
                if ret and frame is not None:
                    # Resize
                    frame = cv2.resize(frame, (cell_w, cell_h), interpolation=cv2.INTER_AREA)
                else:
                    frame = np.zeros((cell_h, cell_w, 3), dtype=np.uint8)
            else:
                # Video ended, show last frame or black? Let's show black.
                frame = np.zeros((cell_h, cell_w, 3), dtype=np.uint8)

            # Place Frame
            canvas[y_offset:y_offset+cell_h, x_offset:x_offset+cell_w] = frame
            
        out_writer.write(canvas)

    out_writer.release()
    for cap in caps: cap.release()
    print(f"Saved stitched video to {output_path}")

def visualize_all_bbox_videos(datanames, unity_output_dir, output_dir='../../outputs/'):
    video_paths = []
    
    for dataname in datanames:
        bbox_path = os.path.join(unity_output_dir, dataname, 'videos', f"{dataname}_normal_bbox.mp4")
        if os.path.isfile(bbox_path):
            video_paths.append(bbox_path)
        else:
            print(f"[Warning] No bbox video found for {dataname}")
            
    if not video_paths:
        print("No videos found.")
        return
        
    output_path = os.path.join(output_dir, "stitched_gallery.mp4")
    # Reduced max columns to 3 for a cleaner look, target width 480p
    stitch_bbox_videos(video_paths, output_path, max_cols=4, target_width=480)

def process_dataname(unity_output_dir, dataname):
    scene_folder = os.path.join(unity_output_dir, dataname)
    frame_dir = os.path.join(scene_folder, '0')
    if not os.path.isdir(frame_dir):
        print(f"[Warning] Skipping: {frame_dir} does not exist.")
        return

    output_dir = os.path.join(scene_folder, 'videos')
    os.makedirs(output_dir, exist_ok=True)

    # Generate source videos first
    for suffix in ['_normal', '_seg_class', '_seg_inst']:
        img_paths = collect_sorted_images(frame_dir, suffix)
        out_path = os.path.join(output_dir, f"{dataname}{suffix}.mp4")
        # Only create if doesn't exist to save time (optional optimization)
        if not os.path.exists(out_path):
            make_video_ffmpeg(img_paths, out_path)

    # Load metadata
    try:
        _, class_list = load_prefab_metadata("../resources/PrefabClass.json")
        instance_colors_path = os.path.join(frame_dir, 'instance_colors.json')
        with open(instance_colors_path, 'r') as f: instance_colors = json.load(f)
        agent_graph_path = os.path.join(frame_dir, 'agent_graph.json')
        with open(agent_graph_path, 'r') as f: agent_graph = json.load(f)
    except Exception as e:
        print(f"Skipping BBox generation for {dataname} due to missing metadata: {e}")
        return

    normal_paths = collect_sorted_images(frame_dir, '_normal')
    cls_paths    = collect_sorted_images(frame_dir, '_seg_class')
    inst_paths   = collect_sorted_images(frame_dir, '_seg_inst')
        
    bbox_out = os.path.join(output_dir, f"{dataname}_normal_bbox.mp4")
    make_bbox_video(normal_paths, inst_paths, instance_colors, agent_graph, cls_paths, class_list, bbox_out, fps=5)

def main():
    args = parse_args()
    if not os.path.isdir(args.unity_output_dir):
        raise FileNotFoundError(f"{args.unity_output_dir} not found.")

    for dataname in tqdm(args.datanames, desc="Processing datanames"):
        process_dataname(args.unity_output_dir, dataname)
        
    if args.make_stitched_video:
        visualize_all_bbox_videos(args.datanames, args.unity_output_dir)

if __name__ == "__main__":
    main()