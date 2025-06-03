
import json
import cv2
import sys
import glob
from tqdm import tqdm

sys.path.append('../simulation')
from unity_simulator.comm_unity import UnityCommunication
from unity_simulator import utils_viz
sys.path.append(".")
from utils_demo import *
from graph_utils import *
from ros_utils import *

def generate_observation(comm):
    with open("../../unity_output/scene4_754ab231d3_0/0/graph.json", "r") as f:
        graph = json.load(f)
    if graph is None:
        raise ValueError("Graph is None, please check the file path or content.")
    comm.reset(4)
    success, message = comm.expand_scene(graph)
    if not success:
        raise RuntimeError(f"Failed to expand scene: {message}")
    
    s, nc_before = comm.camera_count()
    prepare_pano_character_camera(comm)
    comm.add_character('chars/Female2')
    s, nc_after = comm.camera_count()
    cameras_select = list(range(nc_before, nc_after))
    pano_camera_select = cameras_select[8:14]
    
    simulation_data_dir = "../../unity_output/scene4_754ab231d3_0/0"
    dataname = "scene4_754ab231d3_0"
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
            
    output_dir = "../../outputs/debug_observations"
    os.makedirs(output_dir, exist_ok=True)

    # Remove all .png files in the output directory if they exist
    for f in glob.glob(os.path.join(output_dir, "*.png")):
        os.remove(f)

    frames = []
    for idx, position in enumerate(tqdm(hip_positions[:50], desc="Processing positions")):
        success = comm.move_character(0, position)
        if not success:
            print(f"Failed to move character to position {position} at frame {i}.")
            continue
        
        ok_img, imgs = comm.camera_image(pano_camera_select, mode="normal")
        assert len(imgs) == len(pano_camera_select)
        view_pil = display_grid_img(imgs, nrows=2)
        view_pil.save(os.path.join(output_dir, f"{idx:06d}.png"))
        # Convert PIL image to numpy array (BGR for OpenCV)
        frame = cv2.cvtColor(np.array(view_pil), cv2.COLOR_RGB2BGR)
        frames.append(frame)

    # Write video
    if frames:
        height, width, _ = frames[0].shape
        video_path = "../../outputs/example_video2.mp4"
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        out = cv2.VideoWriter(video_path, fourcc, 10, (width, height))
        for frame in frames:
            out.write(frame)
        out.release()

if __name__ == "__main__":
    comm = UnityCommunication(port="8080")
    comm.timeout_wait = 300 
    generate_observation(comm)