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
    SemanticObjectDetectionSrv,
    SemanticObjectDetectionSrvRequest,
)

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
    view_pil.save("../../outputs/debug_observe.png")
    
    rospy.wait_for_service("/owlv2/semantic_object_detection")
    for img in imgs:
        ros_img = opencv_to_ros_image(img)
        try:
            detect_service = rospy.ServiceProxy("/owlv2/semantic_object_detection", SemanticObjectDetectionSrv)
            req = SemanticObjectDetectionSrvRequest()
            req.query_image = ros_img
            req.query_text = "book"
            response = detect_service(req)
        except:
            print("Detection service failed")
            continue
        if len(response.bounding_boxes.bboxes) == 0:
            continue
        import pdb; pdb.set_trace()
        # TODO: The object detector is having trouble detecting the book

if __name__ == "__main__":
    comm = UnityCommunication(port="8080")
    comm.timeout_wait = 300 
    example(comm)