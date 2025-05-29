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
    FindObjectSrv,
    FindObjectSrvResponse,
    SemanticObjectDetectionSrv,
    SemanticObjectDetectionSrvRequest,
    SemanticObjectDetectionSrvResponse
)

comm = None
pano_camera_select = None

def parse_args():
    parser = argparse.ArgumentParser(description='Virtual Home ROS Service')
    parser.add_argument("--graph_path", type=str, required=True, help="Path to the scene graph")
    return parser.parse_args()

### Helper Functions ###
def detect_objects_owlv2(query_image: Image, query_cls: str) -> SemanticObjectDetectionSrvResponse:
    """
    Detect objects by class.
    """
    rospy.wait_for_service("/owlv2/semantic_object_detection")
    try:
        detect_service = rospy.ServiceProxy("/owlv2/semantic_object_detection", SemanticObjectDetectionSrv)
        req = SemanticObjectDetectionSrvRequest()
        req.query_image = query_image
        req.query_text = query_cls
        response = detect_service(req)
        return response
    except rospy.ServiceException as e:
        print("Service call failed:", e)

def observe():
    (ok_img, imgs) = comm.camera_image(pano_camera_select, mode="normal")
    
    view_pil = display_grid_img(imgs, nrows=2)
    view_pil.save("../../outputs/debug_observe.png")
    
    ros_images = []
    for img in imgs:
        ros_img = opencv_to_ros_image(img)
        ros_images.append(ros_img)
    
    return ros_images

### Handle Service Requests ###
def handle_navigate_request(req):
    global comm
    rospy.loginfo("Received navigate request")
    
    success = comm.move_character(0, [req.x, req.y, req.z])
    return GetImageAtPoseSrvResponse(success=success)

def handle_observe_request(req):
    global comm
    rospy.loginfo("Received observe request")
    
    ros_images = observe()
    return GetImageSrvResponse(
        image=ros_images[0], 
        pano_images=ros_images,
    )
    
def handle_visible_objects_request(req):
    global comm
    rospy.loginfo("Received visible objects request")
    
    unique_ids = set()
    for cam in pano_camera_select:
        _, visible_objects = comm.get_visible_objects(cam)
        for obj_id in visible_objects.keys():
            unique_ids.add(int(obj_id))
    
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
    
    target_node_id = None
    for cam in pano_camera_select:
        _, visible_objects = comm.get_visible_objects(cam)
        for node_id, cls_name in visible_objects.items():
            # TODO: need to use VLM to determine the right query_text
            if cls_name.lower() == req.query_text.lower():
                target_node_id = int(node_id)
                break
    success = (target_node_id is not None)
    return FindObjectSrvResponse(
        success=success,
        id=target_node_id,
    )
    
    ros_images = observe()
    for ros_image in ros_images:
        detection_response = detect_objects_owlv2(ros_image, req.query_text)
        for detection in detection_response.bounding_boxes.bboxes:
            xyxy = [int(x) for x in xyxy]
            center_x = (xyxy[0] + xyxy[2]) // 2
            center_y = (xyxy[1] + xyxy[3]) // 2 # TODO
            
    
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
        
    s, nc_before = comm.camera_count()
    prepare_pano_character_camera(comm)
    comm.add_character('chars/Female2', initial_room='bathroom')
    s, nc_after = comm.camera_count()
    cameras_select = list(range(nc_before, nc_after))
    pano_camera_select = cameras_select[8:14]
    
    rospy.Service('/moma/navigate', GetImageAtPoseSrv, handle_navigate_request)
    rospy.loginfo("Ready to navigate")
    rospy.Service('/moma/observe', GetImageSrv, handle_observe_request)
    rospy.loginfo("Ready to observe")
    rospy.Service('/moma/visible_objects', GetVisibleObjectsSrv, handle_visible_objects_request)
    rospy.loginfo("Ready to return visible objects")
    rospy.Service('/moma/find_object', FindObjectSrv, handle_find_request)
    rospy.loginfo("Ready to find objects")
    
    rospy.spin()