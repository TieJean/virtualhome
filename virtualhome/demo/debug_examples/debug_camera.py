from tqdm import tqdm
import sys, os
import json

sys.path.append('../simulation')
from unity_simulator.comm_unity import UnityCommunication
from unity_simulator import utils_viz
sys.path.append(".")
from utils_demo import *
from graph_utils import *

def example(comm):
    with open("../../unity_output/scene4_754ab231d3_0/0/graph.json", "r") as f:
        graph = json.load(f)
    if graph is None:
        raise ValueError("Graph is None, please check the file path or content.")
    comm.reset()
    success, message = comm.expand_scene(graph)
    if not success:
        raise RuntimeError(f"Failed to expand scene: {message}")
    
    s, nc_before = comm.camera_count()
    s, msg = comm.add_character_camera(position=[0, 1.8,  0.0], rotation=[20,  0, 0], field_view=60, name="pano_0")
    s, msg = comm.add_character_camera(position=[0, 1.8,  0.0], rotation=[20, 60, 0], field_view=60, name="pano_1")
    s, msg = comm.add_character_camera(position=[0, 1.8,  0.0], rotation=[20, 120, 0], field_view=60, name="pano_2")
    s, msg = comm.add_character_camera(position=[0, 1.8,  0.0], rotation=[20, 180, 0], field_view=60, name="pano_3")
    s, msg = comm.add_character_camera(position=[0, 1.8,  0.0], rotation=[20, 240, 0], field_view=60, name="pano_4")
    s, msg = comm.add_character_camera(position=[0, 1.8,  0.0], rotation=[20, 300, 0], field_view=60, name="pano_5")
    comm.add_character('chars/Female2')
    s, nc_after = comm.camera_count()
    cameras_select = list(range(nc_before, nc_after))
    print(nc_before, nc_after)
    
    (ok_img, imgs) = comm.camera_image(cameras_select, mode="normal")
    view_pil = display_grid_img(imgs, nrows=4)
    view_pil.save("../../outputs/debug_observe.png")
    

if __name__ == "__main__":
    comm = UnityCommunication(port="8080")
    comm.timeout_wait = 300 
    
    example(comm)