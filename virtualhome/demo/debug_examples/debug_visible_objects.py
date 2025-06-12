from tqdm import tqdm
import sys, os
import json
import argparse
import copy

sys.path.append('../simulation')
from unity_simulator.comm_unity import UnityCommunication
from unity_simulator import utils_viz
sys.path.append(".")
from utils_demo import *
from graph_utils import *

def debug_visible_objects(comm, pos: list):
    s, nc_before = comm.camera_count()
    prepare_pano_character_camera(comm)
    comm.add_character('chars/Female2', initial_room='bathroom')
    s, nc_after = comm.camera_count()
    cameras_select = list(range(nc_before, nc_after))
    pano_camera_select = cameras_select[8:14]  # Panoramic cameras
    
    comm.add_character('chars/Female2', initial_room='bathroom')
    
    success = comm.move_character(0, pos)
    if not success:
        raise RuntimeError(f"Failed to move character to position {pos}")
    
    visible_by_camera = {}
    for cam_idx, cam_id in enumerate(pano_camera_select):
        _, visible_objects = comm.get_visible_objects(cam_id)
        visible_classes = []
        for node in graph["nodes"]:
            if node["id"] in visible_objects.keys():
                visible_classes.append(node["class_name"])
                
    ok_img, imgs = comm.camera_image(pano_camera_select, mode="normal")
    view_pil = display_grid_img(imgs, nrows=2)
    view_pil.save("../../outputs/debug_pano.png")
    
    ok_img, imgs = comm.camera_image([pano_camera_select[0]-6], mode="normal")
    view_pil = display_grid_img(imgs, nrows=1)
    view_pil.save("../../outputs/debug_first_person.png")

if __name__ == "__main__":
    comm = UnityCommunication(port="8080")
    comm.timeout_wait = 300
    
    graph_path = "../../unity_output/scene4_a239c9b7fc_0/0/graph.json"
    with open(graph_path, "r") as f:
        graph = json.load(f)
    if graph is None:
        raise ValueError(f"Failed to load graph from {graph_path}")
    
    comm.reset(4)
    success, message = comm.expand_scene(graph)
    if not success:
        print(f"Failed to load scene from {graph_path} to the simulator:", message)
        sys.exit(1)
        
    pose_path = "../../unity_output/scene4_a239c9b7fc_0/0/pd_scene4_a239c9b7fc_0.txt"
    positions = []
    with open(pose_path, "r") as f:
        lines = f.readlines()
        for line in lines[1:]:
            values = line.strip().split()
            if len(values) < 4:
                continue
            # x, y, z = map(float, values[1+10*3:4+10*3])
            # positions.append([x, y, z])
            x1, y1, z1 = map(float, values[1+5*3:4+5*3])
            x2, y2, z2 = map(float, values[1+6*3:4+6*3])
            positions.append([(x1+x2)/2, (y1+y2)/2, ((z1+z2)/2)])
    # import pdb; pdb.set_trace()
        
    pos = positions[90]
    # pos = [2.502059, 0.9212801, -7.795268]
    # pos = [3.34, 0.87, -4.50]
    debug_visible_objects(comm, pos)
        
        