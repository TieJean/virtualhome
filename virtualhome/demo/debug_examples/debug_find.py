from tqdm import tqdm
import sys, os
import json
import rospy

sys.path.append('../simulation')
from unity_simulator.comm_unity import UnityCommunication
from unity_simulator import utils_viz
sys.path.append(".")
from utils_demo import *
from graph_utils import *
from ros_utils import *

import roslib; roslib.load_manifest('amrl_msgs')
from amrl_msgs.srv import (
    FindObjectSrvResponse,
)

def example_1(comm):
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
    
    position = [2.625752, 0.8725489, -10.35497]
    success = comm.move_character(0, position)
    if not success:
        raise RuntimeError("Failed to move character to the specified position.")
    
    (ok_img, imgs) = comm.camera_image(pano_camera_select, mode="normal")
    view_pil = display_grid_img(imgs, nrows=2)
    view_pil.save("../../outputs/debug.png")
    
    query_text = "book"
    
    target_node_id = None
    for cam in pano_camera_select:
        _, visible_objects = comm.get_visible_objects(cam)
        for node_id, cls_name in visible_objects.items():
            if cls_name.lower() == query_text.lower():
                target_node_id = int(node_id)
                break
                
    success = (target_node_id is not None)
    response = FindObjectSrvResponse(
        success=success,
        id=target_node_id,
    )
    print(response)


if __name__ == "__main__":
    comm = UnityCommunication(port="8080")
    comm.timeout_wait = 300 
    example_1(comm)