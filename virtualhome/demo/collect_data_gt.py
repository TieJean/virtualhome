from tqdm import tqdm
import os, json, argparse
import sys
import cv2

sys.path.append('../simulation')
from unity_simulator.comm_unity import UnityCommunication
from utils_demo import *
from graph_utils import *

def parse_args():
    parser = argparse.ArgumentParser(description="Post-process collected data for VirtualHome.")
    parser.add_argument('--data_dir', type=str, default="../../unity_output", help='Directory to save the processed data.')
    parser.add_argument('--datanames', nargs='+', type=str, required=True, help='List of data names to process.')
    return parser.parse_args()

def postprocess_visibility_once(comm, data_dir: str, dataname: str):
    print(f"\n🔧 Processing: {dataname}")
    simulation_data_dir = os.path.join(data_dir, "0")

    # Load pose file
    pose_path = os.path.join(simulation_data_dir, f"pd_{dataname}.txt")
    if not os.path.isfile(pose_path):
        raise FileNotFoundError(f"Missing pose file: {pose_path}")
    
    hip_positions = []
    with open(pose_path, "r") as f:
        lines = f.readlines()
        for line in lines[1:]:
            values = line.strip().split()
            if len(values) < 4:
                continue
            x, y, z = map(float, values[1:4])
            hip_positions.append([x, y, z])
    
    comm.add_character('chars/Female2', initial_room='bathroom')

    # Set up cameras
    _, nc = comm.camera_count()
    char_cam_indices = range(nc - 6, nc)
    cameras_select = [list(range(nc))[i] for i in char_cam_indices]

    # Setup directories
    image_dir = os.path.join(data_dir, "images")
    right_image_dir = os.path.join(data_dir, "images_right")
    left_image_dir = os.path.join(data_dir, "images_left")
    back_image_dir = os.path.join(data_dir, "images_back")
    for d in [image_dir, right_image_dir, left_image_dir, back_image_dir]:
        os.makedirs(d, exist_ok=True)
        for root, _, files in os.walk(image_dir):
            for file in files:
                filepath = os.path.join(root, file)
                os.remove(filepath)

    # Capture and annotate
    frame_data = []
    for i, position in enumerate(tqdm(hip_positions, total=len(hip_positions))):
        success = comm.move_character(0, position)
        if not success:
            print(f"Failed to move character to position {position} at frame {i}.")
            continue
            # raise RuntimeError(f"Failed to move character to position {position} at frame {i}. Double-check the imported graph is correct.")
        
        (ok_img, imgs) = comm.camera_image(cameras_select, mode="normal")
        img, right_img, left_img, back_img = imgs[0], imgs[3], imgs[4], imgs[5]

        cv2.imwrite(os.path.join(image_dir, f"{i:06d}.png"), img)
        cv2.imwrite(os.path.join(right_image_dir, f"{i:06d}.png"), right_img)
        cv2.imwrite(os.path.join(left_image_dir, f"{i:06d}.png"), left_img)
        cv2.imwrite(os.path.join(back_image_dir, f"{i:06d}.png"), back_img)

        _, front_visible_objects = comm.get_visible_objects(cameras_select[0])
        _, right_visible_objects = comm.get_visible_objects(cameras_select[3])
        _, left_visible_objects = comm.get_visible_objects(cameras_select[4])
        _, back_visible_objects = comm.get_visible_objects(cameras_select[5])

        frame_data.append([
            i,
            {
                "front": list(front_visible_objects.keys()),
                "right": list(right_visible_objects.keys()),
                "left": list(left_visible_objects.keys()),
                "back": list(back_visible_objects.keys()),
            }
        ])
    
    # Save ground-truth annotations
    success, graph = comm.environment_graph()
    gt_path = os.path.join(simulation_data_dir, "gt_annotations.json")
    with open(gt_path, "w") as f:
        json.dump({
            "frames": frame_data,
            "graph": graph
        }, f, indent=2)
    print(f"✅ Saved visibility annotations: {gt_path}")


def run(args):
    # Reconnect to simulator
    comm = UnityCommunication(port="8080")
    comm.timeout_wait = 300
    
    for dataname in args.datanames:
        data_dir = os.path.join(args.data_dir, dataname)
        
        graph_path = os.path.join(data_dir, "0", "graph.json")
        with open(graph_path, "r") as f:
            graph = json.load(f)
        if not graph:
            raise ValueError(f"Graph data is empty for {dataname}. Please ensure the graph is generated before post-processing.")
        
        comm.reset()
        success, message = comm.expand_scene(graph)
        if not success:
            raise RuntimeError(f"Failed to expand scene for {dataname}: {message}")
        
        postprocess_visibility_once(comm, data_dir, dataname)


if __name__ == "__main__":
    args = parse_args()
    run(args)