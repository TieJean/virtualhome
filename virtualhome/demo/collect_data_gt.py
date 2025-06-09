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

def postprocess_visibility_from_segcls_once(comm, data_dir: str, dataname: str):
    print(f"\n🔧 Processing: {dataname}")
    s, nc_before = comm.camera_count()
    prepare_pano_character_camera(comm)
    comm.add_character('chars/Female2', initial_room='bathroom')
    s, nc_after = comm.camera_count()
    # 2 - is the ego centric view
    # 5:8 - are right, left and back cameras
    # 8:14 - are the panoramic cameras added thru `prepare_pano_character_camera`
    cameras_select = list(range(nc_before, nc_after))
    pano_camera_select = cameras_select[8:14]  # Panoramic cameras
    
    simulation_data_dir = os.path.join(data_dir, "0")
    
    image_dir = os.path.join(data_dir, "images")
    os.makedirs(image_dir, exist_ok=True)
    for root, _, files in os.walk(image_dir):
        for file in files:
            os.remove(os.path.join(root, file))

    prefab_classes, class_list = load_prefab_metadata("../resources/PrefabClass.json")

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
            
    seg_class_files = sorted([
        f for f in os.listdir(simulation_data_dir)
        if f.endswith('_seg_class.png')
    ])
    normal_files = sorted([
        f for f in os.listdir(simulation_data_dir)
        if f.endswith('_normal.png')
    ])
    depth_files = sorted([
        f for f in os.listdir(simulation_data_dir)
        if f.endswith('_depth.exr')
    ])
    assert len(seg_class_files) == len(hip_positions), (
        f"Mismatch: {len(seg_class_files)} seg_class files vs {len(hip_positions)} hip positions"
    )
    assert len(seg_class_files) == len(normal_files), (
        f"Mismatch: {len(seg_class_files)} seg_class files vs {len(normal_files)} normal files"
    )
    assert len(seg_class_files) == len(depth_files), (
        f"Mismatch: {len(depth_files)} depth files vs {len(normal_files)} normal files"
    )
    assert len(seg_class_files) == len(hip_positions), (
        f"Mismatch: {len(seg_class_files)} seg_class files vs {len(hip_positions)} hip positions"
    )
    
    frame_data = []
    for seg_cls_filename, normal_filename, depth_filename, position in tqdm(zip(seg_class_files, normal_files, depth_files, hip_positions), total=len(seg_class_files), desc="Processing frames"):
        idx = normal_files.index(normal_filename)
        
        success = comm.move_character(0, position)
        if not success:
            print(f"Failed to move character to position {position} at frame {idx}.")
            continue
        
        depth_path = os.path.join(simulation_data_dir, depth_filename)
        depth_img = iio.imread(depth_path)
        assert depth_img.dtype in (np.float32, np.float64), f"Unexpected dtype: {depth_img.dtype}"
        
        # Save the normal image to the images directory with a zero-padded filename
        normal_path = os.path.join(simulation_data_dir, normal_filename)
        normal_img = cv2.imread(normal_path)
        if normal_img is None:
            raise ValueError(f"Failed to read normal image: {normal_path}")
        output_filename = os.path.join(image_dir, f"{idx:06d}.png")
        cv2.imwrite(output_filename, normal_img)
        
        img_path = os.path.join(simulation_data_dir, seg_cls_filename)
        bgr_img = cv2.imread(img_path)  # BGR format
        if bgr_img is None:
            raise ValueError(f"Failed to read image: {img_path}")
        # Convert to RGB
        rgb_img = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2RGB)
        # Get scalar depth map
        if depth_img.ndim == 3 and depth_img.shape[2] == 4:
            depth_scalar_img = depth_img[..., 0]
        else:
            depth_scalar_img = depth_img
        # Mask to only pixels where depth < 2.0
        valid_mask = (depth_scalar_img < 2.0)
        # Apply depth mask to RGB segmentation image
        rgb_masked = rgb_img[valid_mask]
        # Remove black pixels and get unique colors
        non_black = rgb_masked[~np.all(rgb_masked == 0, axis=1)]
        unique_colors = np.unique(non_black, axis=0)
        
        # flat_pixels = rgb_img.reshape(-1, 3)
        # unique_colors = np.unique(flat_pixels, axis=0)
        # unique_colors = [tuple(color) for color in unique_colors if not np.all(color == 0)]

        # Map each color to class name
        detected_classes = []
        for color in unique_colors:
            class_name = semantic_rgb_to_cls(color, class_list)
            detected_classes.append(class_name)
            
        # Visibility annotations per pano view
        visible_by_camera = {}
        for cam_idx, cam_id in enumerate(pano_camera_select):
            _, visible_objects = comm.get_visible_objects(cam_id)
            visible_by_camera[f"pano_{cam_idx}"] = list(visible_objects.keys())
        
        frame_data.append(
            [idx, visible_by_camera, detected_classes]
        )
    
    # Save ground-truth annotations
    success, graph = comm.environment_graph()
    gt_path = os.path.join(simulation_data_dir, "gt_annotations.json")
    with open(gt_path, "w") as f:
        json.dump({
            "frames": frame_data,
            "graph": graph
        }, f, indent=2)
    print(f"✅ Saved visibility annotations: {gt_path}")
    
    
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
        
        # postprocess_visibility_once(comm, data_dir, dataname)
        postprocess_visibility_from_segcls_once(comm, data_dir, dataname)


if __name__ == "__main__":
    args = parse_args()
    run(args)