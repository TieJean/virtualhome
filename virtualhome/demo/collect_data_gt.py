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
    
    s, nc_before = comm.camera_count()
    prepare_pano_character_camera(comm)
    comm.add_character('chars/Female2', initial_room='bathroom')
    s, nc_after = comm.camera_count()
    # 2 - is the ego centric view
    # 5:8 - are right, left and back cameras
    # 8:14 - are the panoramic cameras added thru `prepare_pano_character_camera`
    cameras_select = list(range(nc_before, nc_after))
    pano_camera_select = cameras_select[8:14]  # Panoramic cameras
    
    # Setup directories
    image_dir = os.path.join(data_dir, "images")
    pano_dirs = [
        os.path.join(data_dir, f"pano_{i}") for i in range(len(pano_camera_select))
    ]
    for d in pano_dirs + [image_dir]:
        os.makedirs(d, exist_ok=True)
        for root, _, files in os.walk(d):
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
        
        ok_img, imgs = comm.camera_image(pano_camera_select, mode="normal")
        assert len(imgs) == len(pano_camera_select)
        
        for cam_idx, pano_img in enumerate(imgs):
            filename = f"{i:06d}.png"
            cv2.imwrite(os.path.join(pano_dirs[cam_idx], filename), pano_img)

            if cam_idx == 0:
                # Also save pano_0 image to "images/"
                cv2.imwrite(os.path.join(image_dir, filename), pano_img)

        # Visibility annotations per pano view
        visible_by_camera = {}
        for cam_idx, cam_id in enumerate(pano_camera_select):
            _, visible_objects = comm.get_visible_objects(cam_id)
            visible_by_camera[f"pano_{cam_idx}"] = list(visible_objects.keys())

        frame_data.append([i, visible_by_camera])
    
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