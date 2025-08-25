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
        help="Path to unity_output directory (default: ../../unity_output/)"
    )
    parser.add_argument(
        "--datanames", 
        type=str, 
        nargs='+', 
        required=True, 
        help="List of dataname folders to process (e.g., scene4_754ab231d3_0)"
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
        CLASS_PALETTE = {
            # "book":     (  0,   0, 255),   # red
            # "folder":   (  0, 255,   0),   # green
            # "toy":      (255,   0,   0),   # blue
            # "magazine": (  0, 255, 255),   # yellow
            "cereal":   (255, 128, 0),     # orange
            "bananas":  (255, 255,   0),   # purple
            "cupcake":  (255, 192, 203),   # pink
            "mincedmeat": (128, 0, 128),   # violet
        }
        TARGET_CLASSES = set(CLASS_PALETTE.keys())
        
        if instance_mask.ndim == 2:  # single channel
            instance_mask = cv2.cvtColor(instance_mask, cv2.COLOR_GRAY2BGR)
        elif instance_mask.shape[2] == 4:  # BGRA
            instance_mask = instance_mask[:, :, :3]
            
        if class_mask.ndim == 2:  # single channel
            class_mask = cv2.cvtColor(class_mask, cv2.COLOR_GRAY2BGR)
        elif class_mask.shape[2] == 4:  # BGRA
            class_mask = class_mask[:, :, :3]

        out = image.copy()

        for node in graph["nodes"]:
            cls_name = node.get("class_name", "").lower()
            if cls_name not in TARGET_CLASSES:
                continue

            uid = str(node["id"])
            rgb_f = instance_colors.get(uid)
            if rgb_f is None:
                continue

            # Convert Unity RGB float [0‑1] -> uint8 BGR
            bgr_uint8 = (
                int(round(rgb_f[2] * 255)),  # B
                int(round(rgb_f[1] * 255)),  # G
                int(round(rgb_f[0] * 255)),  # R
            )

            # Binary mask for instance color
            mask_instance = cv2.inRange(instance_mask, np.array(bgr_uint8), np.array(bgr_uint8))
            # Binary mask for class color
            bgr_class = class_colors[cls_name]
            mask_class = cv2.inRange(class_mask, np.array(bgr_class), np.array(bgr_class))
            # Final mask: only pixels where both match
            mask = cv2.bitwise_and(mask_instance, mask_class)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if not contours:
                continue

            color = CLASS_PALETTE.get(cls_name, (255, 255, 255))  # fallback white
            for cnt in contours:
                x, y, w, h = cv2.boundingRect(cnt)
                if w < 4 or h < 4:        # ignore tiny specks
                    continue
                cv2.rectangle(out, (x, y), (x + w, y + h), color, 2)
                cv2.putText(
                    out,
                    cls_name,
                    (x, y - 6),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    color,
                    1,
                    cv2.LINE_AA,
                )

        return out
    
    """
    Draw bounding‑boxes (via your `_draw_bounding_box`) on each normal/instance
    pair and encode them into a video.  Uses `make_video_ffmpeg` unchanged.
    """
    if not normal_paths or not inst_paths or not cls_paths or len(normal_paths) != len(inst_paths) or len(normal_paths) != len(cls_paths):
        print(f"[Warning] Bounding‑box video skipped for {out_path} (frame mismatch).")
        return

    class_colors = {}
    for cls_name in ["book", "folder", "toy", "magazine", "bananas", "cupcake", "cereal", "mincedmeat"]:
        class_colors[cls_name] = semantic_cls_to_bgr(cls_name, class_list)

    tmp_dir = tempfile.mkdtemp()              # store annotated PNGs here
    try:
        for idx, (n_path, i_path, c_path) in enumerate(zip(normal_paths, inst_paths, cls_paths)):
            img_normal = cv2.imread(n_path)                           # BGR
            img_inst   = cv2.imread(i_path, cv2.IMREAD_UNCHANGED)     # seg‑inst
            img_cls    = cv2.imread(c_path, cv2.IMREAD_UNCHANGED)     # seg‑class
            drawn      = _draw_bounding_box(img_normal, img_inst, img_cls, instance_colors, class_colors)
            cv2.imwrite(os.path.join(tmp_dir, f"frame_{idx:04d}_bbox.png"), drawn)

        annotated_frames = sorted(glob(os.path.join(tmp_dir, "frame_*.png")))
        make_video_ffmpeg(annotated_frames, out_path, fps=fps)        # ← untouched
    finally:
        shutil.rmtree(tmp_dir, ignore_errors=True)

def make_video_ffmpeg(image_paths, out_path, fps=5):
    if not image_paths:
        print(f"[Warning] No images found for {out_path}")
        return

    folder = os.path.dirname(image_paths[0])
    sample_name = os.path.basename(image_paths[0])
    parts = sample_name.split('_')

    if len(parts) < 3:
        print(f"[Error] Unexpected filename format: {sample_name}")
        return

    # prefix = parts[0]  # e.g., frame
    # suffix = parts[-1].replace('.png', '')  # e.g., normal
    # pattern = os.path.join(folder, f"{prefix}_%04d_{suffix}.png")
    
    # ── derive pattern:  frame_%04d_seg_inst.png  (works with seg_class etc.) ──
    try:
        prefix, _, remainder = sample_name.split('_', 2)  # "frame", "0000", "seg_inst.png"
    except ValueError:
        print(f"[Error] Unexpected filename format: {sample_name}")
        return

    suffix = remainder.rsplit('.', 1)[0]                  # "seg_inst"  (or "normal")
    pattern = os.path.join(folder, f"{prefix}_%04d_{suffix}.png")

    os.makedirs(os.path.dirname(out_path), exist_ok=True)

    command = [
        'ffmpeg',
        '-y',
        '-framerate', str(fps),
        '-i', pattern,
        '-c:v', 'libx264',
        '-pix_fmt', 'yuv420p',
        '-r', str(fps),
        out_path
    ]

    try:
        subprocess.run(command, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        print(f"Saved video to {out_path}")
    except subprocess.CalledProcessError:
        print(f"[Error] ffmpeg failed for {out_path}")
        
def stitch_bbox_videos(video_paths, output_path, grid_cols=4):
    import math

    # Open all video captures
    caps = [cv2.VideoCapture(p) for p in video_paths]
    n_videos = len(caps)
    if n_videos == 0:
        print(f"No videos to stitch for {output_path}")
        return

    # Video properties (use the first video for shape/fps)
    width = int(caps[0].get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(caps[0].get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = caps[0].get(cv2.CAP_PROP_FPS)

    # Determine max frame count among all videos
    frame_counts = [int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) for cap in caps]
    max_frames = max(frame_counts)

    # Calculate grid size
    grid_rows = math.ceil(n_videos / grid_cols)
    grid_w = width * grid_cols
    grid_h = height * grid_rows

    # Output writer
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out_writer = cv2.VideoWriter(output_path, fourcc, fps, (grid_w, grid_h))

    # For each frame index up to max_frames:
    for frame_idx in range(max_frames):
        frames = []
        for vid_i, cap in enumerate(caps):
            # If frame_idx < this video's length, read frame
            if frame_idx < frame_counts[vid_i]:
                ret, frame = cap.read()
                if not ret or frame is None:
                    frame = np.zeros((height, width, 3), dtype=np.uint8)
                elif frame.shape[2] == 4:
                    frame = frame[..., :3]  # Remove alpha
            else:
                # This video finished: black frame
                frame = np.zeros((height, width, 3), dtype=np.uint8)
            frames.append(frame)
        # Pad if needed
        while len(frames) < grid_rows * grid_cols:
            frames.append(np.zeros((height, width, 3), dtype=np.uint8))
        # Build the grid
        grid_img = []
        for i in range(grid_rows):
            row = np.concatenate(frames[i*grid_cols:(i+1)*grid_cols], axis=1)
            grid_img.append(row)
        grid_img = np.concatenate(grid_img, axis=0)
        out_writer.write(grid_img)
    out_writer.release()
    for cap in caps:
        cap.release()
    print(f"Saved stitched video to {output_path}")

def visualize_all_bbox_videos(datanames, unity_output_dir, output_dir='../../outputs/'):
    # Collect bbox video paths from all datanames
    video_paths = []
    for dataname in datanames:
        bbox_path = os.path.join(unity_output_dir, dataname, 'videos', f"{dataname}_normal_bbox.mp4")
        if os.path.isfile(bbox_path):
            video_paths.append(bbox_path)
        else:
            print(f"[Warning] No bbox video found for {dataname} at {bbox_path}")
    if not video_paths:
        print("No bbox videos found to stitch.")
        return
    output_path = os.path.join(output_dir, "stitched_bbox_grid.mp4")
    stitch_bbox_videos(video_paths, output_path)

def process_dataname(unity_output_dir, dataname):
    scene_folder = os.path.join(unity_output_dir, dataname)
    frame_dir = os.path.join(scene_folder, '0')
    if not os.path.isdir(frame_dir):
        print(f"[Warning] Skipping: {frame_dir} does not exist.")
        return

    output_dir = os.path.join(scene_folder, 'videos')
    os.makedirs(output_dir, exist_ok=True)

    for suffix in ['_normal', '_seg_class', '_seg_inst']:
        img_paths = collect_sorted_images(frame_dir, suffix)
        out_path = os.path.join(output_dir, f"{dataname}{suffix}.mp4")
        make_video_ffmpeg(img_paths, out_path)

    _, class_list = load_prefab_metadata("../resources/PrefabClass.json")

    normal_paths = collect_sorted_images(frame_dir, '_normal')
    cls_paths    = collect_sorted_images(frame_dir, '_seg_class')
    inst_paths   = collect_sorted_images(frame_dir, '_seg_inst')
    
    instance_colors_path = os.path.join(frame_dir, 'instance_colors.json')
    with open(instance_colors_path, 'r') as f:
        instance_colors = json.load(f)
    if not instance_colors:
        raise ValueError(f"No instance colors found in {instance_colors_path}")
    agent_graph_path = os.path.join(frame_dir, 'agent_graph.json')
    with open(agent_graph_path, 'r') as f:
        agent_graph = json.load(f)
    if not agent_graph:
        raise ValueError(f"No agent graph found in {agent_graph_path}")
        
    bbox_out = os.path.join(output_dir, f"{dataname}_normal_bbox.mp4")
    make_bbox_video(normal_paths, 
                    inst_paths, 
                    instance_colors, 
                    agent_graph, 
                    cls_paths, 
                    class_list,
                    bbox_out, 
                    fps=5)

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
