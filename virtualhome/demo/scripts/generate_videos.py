import argparse
import os
import subprocess
from glob import glob
from tqdm import tqdm

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
    return parser.parse_args()

def collect_sorted_images(folder, suffix):
    return sorted(glob(os.path.join(folder, f'*{suffix}.png')))

def make_video_ffmpeg(image_paths, out_path, fps=10):
    if not image_paths:
        print(f"[Warning] No images found for {out_path}")
        return

    folder = os.path.dirname(image_paths[0])
    sample_name = os.path.basename(image_paths[0])
    parts = sample_name.split('_')

    if len(parts) < 3:
        print(f"[Error] Unexpected filename format: {sample_name}")
        return

    prefix = parts[0]  # e.g., frame
    suffix = parts[-1].replace('.png', '')  # e.g., normal
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

def main():
    args = parse_args()

    if not os.path.isdir(args.unity_output_dir):
        raise FileNotFoundError(f"{args.unity_output_dir} not found.")

    for dataname in tqdm(args.datanames, desc="Processing datanames"):
        process_dataname(args.unity_output_dir, dataname)

if __name__ == "__main__":
    main()
