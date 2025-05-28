import argparse
import json
import sys

# Simulation
sys.path.append('../simulation')
from unity_simulator.comm_unity import UnityCommunication
from unity_simulator import utils_viz
from ros_utils import *
from utils_demo import *
from graph_utils import *

## ROS Service Calls
import rospy
import cv2
import roslib; roslib.load_manifest('amrl_msgs')
from amrl_msgs.srv import (
    GetImageSrv,
    GetImageSrvResponse,
    GetImageAtPoseSrv, 
    GetImageAtPoseSrvResponse, 
    PickObjectSrv, 
    PickObjectSrvResponse,
    GetVisibleObjectsSrv,
    GetVisibleObjectsSrvResponse,
)

comm = None
cameras_select = None

def parse_args():
    parser = argparse.ArgumentParser(description='Virtual Home ROS Service')
    parser.add_argument("--graph_path", type=str, required=True, help="Path to the scene graph")
    return parser.parse_args()

def observe():
    (ok_img, imgs) = comm.camera_image(cameras_select, mode="normal")
    
    view_pil = display_grid_img([imgs[0], imgs[3], imgs[4], imgs[5]], nrows=2)
    view_pil.save("../../outputs/debug_observe.png")
    
    ros_images = {}
    ros_images["image"] = opencv_to_ros_image(imgs[0])
    ros_images["right"] = opencv_to_ros_image(imgs[3])
    ros_images["left"] = opencv_to_ros_image(imgs[4])
    ros_images["back"] = opencv_to_ros_image(imgs[5])
    
    return ros_images

def handle_navigate_request(req):
    global comm
    rospy.loginfo("Received navigate request")
    
    success = comm.move_character(0, [req.x, req.y, req.z])
    
    if False:
        ros_images = observe()
        return GetImageAtPoseSrvResponse(
            success=success, 
            image=ros_images["image"], 
            right=ros_images["right"], 
            left=ros_images["left"], 
            back=ros_images["back"]
        )
    return GetImageAtPoseSrvResponse(success=success)

def handle_observe_request(req):
    global comm
    rospy.loginfo("Received observe request")
    
    # Following `get_scene_cameras` function
    ros_images = observe()
    return GetImageSrvResponse(
        image=ros_images["image"], 
        right=ros_images["right"], 
        left=ros_images["left"], 
        back=ros_images["back"]
    )
    
def handle_visible_objects_request(req):
    global comm
    rospy.loginfo("Received visible objects request")
    
    _, front_visible_objects = comm.get_visible_objects(cameras_select[0])
    _, right_visible_objects = comm.get_visible_objects(cameras_select[3])
    _, left_visible_objects = comm.get_visible_objects(cameras_select[4])
    _, back_visible_objects = comm.get_visible_objects(cameras_select[5])
    
    # Aggregate all unique IDs
    unique_ids = set()
    for cam_view in [front_visible_objects, right_visible_objects, left_visible_objects, back_visible_objects]:
        for obj_id in cam_view.keys():
            unique_ids.add(int(obj_id))  # Cast to int in case it's str
    nodes = extract_nodes_by_ids(graph["nodes"], unique_ids)
    
    # Format return values
    ids = [int(node["id"]) for node in nodes]
    classnames = [node["class_name"] for node in nodes]
    prefabnames = [node["prefab_name"] for node in nodes]

    return GetVisibleObjectsSrvResponse(
        ids=ids,
        classnames=classnames,
        prefabnames=prefabnames
    )
    
def handle_find_request(req):
    global comm
    rospy.loginfo("Received find request")
    # TODO
    
def handle_pick_request(req):
    global comm
    rospy.loginfo("Received pick request")
    # TODO

if __name__ == "__main__":
    rospy.init_node('virtualhome_ros', anonymous=True)
    
    args = parse_args()
    
    comm = UnityCommunication()
    comm.timeout_wait = 300
    with open(args.graph_path, "r") as f:
        graph = json.load(f)
    if graph is None:
        raise ValueError(f"Failed to load graph from {args.graph_path}")
    
    comm.reset()
    success, message = comm.expand_scene(graph)
    if not success:
        print(f"Failed to load scene from {args.graph_path} to the simulator:", message)
        sys.exit(1)
    comm.add_character('chars/Male2', initial_room='bathroom')
    s, nc = comm.camera_count()
    char_cam_indices = range(nc - 6, nc) # 0 should be ego centric
    _, ncameras = comm.camera_count()
    cameras_select = list(range(ncameras))
    cameras_select = [cameras_select[x] for x in char_cam_indices]
    
    rospy.Service('/moma/navigate', GetImageAtPoseSrv, handle_navigate_request)
    rospy.loginfo("Ready to navigate")
    rospy.Service('/moma/observe', GetImageSrv, handle_observe_request)
    rospy.loginfo("Ready to observe")
    rospy.Service('/moma/visible_objects', GetVisibleObjectsSrv, handle_visible_objects_request)
    rospy.loginfo("Ready to return visible objects")
    
    rospy.spin()