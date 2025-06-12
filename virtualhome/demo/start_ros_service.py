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
    SemanticObjectDetectionSrvResponse,
    ChangeVirtualHomeGraphSrv,
    ChangeVirtualHomeGraphSrvResponse,
)

comm = None
class_list = None
cameras_select = None
pano_camera_select = None

def parse_args():
    parser = argparse.ArgumentParser(description='Virtual Home ROS Service')
    # parser.add_argument("--graph_path", type=str, required=True, help="Path to the scene graph")
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
    rospy.loginfo(f"Received navigate request: ({req.x}, {req.y}, {req.z})")
    
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
    
    success, graph = comm.environment_graph()
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
    
def find_target_node_id(query_text):
    depth_thresh = 3.0
    
    target_node_id = None
    
    for _ in range(6):
        ok_img, normal_imgs = comm.camera_image(cameras_select[2:3], mode="normal")
        ok_img, cls_imgs = comm.camera_image(cameras_select[2:3], mode="seg_class")
        ok_img, depth_imgs = comm.camera_image(cameras_select[2:3], mode="depth")
        normal_img = normal_imgs[0]
        cls_img = cls_imgs[0]
        
        depth_img = depth_imgs[0]
        if depth_img.ndim == 3 and depth_img.shape[2] == 4:
            depth_scalar_img = depth_img[..., 0]
        else:
            depth_scalar_img = depth_img
        valid_mask = (depth_scalar_img < depth_thresh)
        bgr_masked = cls_img[valid_mask]
        
        target_color = semantic_cls_to_bgr(query_text, class_list)
        match_mask = np.all(bgr_masked == target_color, axis=-1)
        display_grid_img(normal_imgs, nrows=1).save("../../outputs/debug.png")
        if np.any(match_mask):
            _, visible_objects = comm.get_visible_objects(cameras_select[2])
            for node_id, cls_name in visible_objects.items():
                if cls_name.lower() == query_text.lower():
                    target_node_id = int(node_id)
                    rospy.loginfo(f"Found target node ID: {target_node_id}")
                    return target_node_id 
    
        script = ["<char0> [TurnRight]", "<char0> [TurnRight]"]
        success, message = comm.render_script(script=script,
                                    processing_time_limit=30,
                                    find_solution=False,
                                    image_width=640,
                                    image_height=480,  
                                    skip_animation=True,
                                    recording=False,
                                    save_pose_data=False)
        if not success:
            rospy.logerr(f"Failed to turn character: {message}")
            return FindObjectSrvResponse(success=False, id=None)
        
    return target_node_id
    
def handle_find_request(req):
    global comm
    rospy.loginfo("Received find request")
    
    target_node_id = find_target_node_id(req.query_text)
    
    return FindObjectSrvResponse(
        success=target_node_id is not None,
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
    
    query_text = req.query_text.lower()
    target_node_id = find_target_node_id(query_text)
    
    if target_node_id is None:
        rospy.logwarn(f"Object '{query_text}' not found in visible objects.")
        return PickObjectSrvResponse(success=False)
    
    script = [f"<char0> [Grab] <{query_text}> ({target_node_id})"]
    success, message = comm.render_script(script=script,
                                        processing_time_limit=60,
                                        find_solution=False,
                                        image_width=640,
                                        image_height=480,  
                                        skip_animation=True,
                                        recording=False,
                                        save_pose_data=False)
    
    success, graph = comm.environment_graph()
    target_node = extract_nodes_by_ids(graph["nodes"], [target_node_id])[0]
    instance_uid = target_node["prefab_name"]
    
    if not success:
        import pdb; pdb.set_trace()
    
    return PickObjectSrvResponse(
        success=success,
        instance_uid=instance_uid
    )
    
def handle_virtualhome_scene_request(req):
    global comm, cameras_select, pano_camera_select
    rospy.loginfo(f"Received change virtual home graph request: {req.graph_path}")
    
    with open(req.graph_path, "r") as f:
        graph = json.load(f)
    
    if graph is None:
        import pdb; pdb.set_trace()
        return ChangeVirtualHomeGraphSrvResponse(success=False)
    
    if req.scene_id is not None:
        comm.reset(req.scene_id)
    else:
        comm.reset()
    success, message = comm.expand_scene(graph)
    if not success:
        import pdb; pdb.set_trace()
        return ChangeVirtualHomeGraphSrvResponse(success=False)
    
    s, nc_before = comm.camera_count()
    prepare_pano_character_camera(comm)
    comm.add_character('chars/Female2', initial_room='bathroom')
    s, nc_after = comm.camera_count()
    cameras_select = list(range(nc_before, nc_after))
    pano_camera_select = cameras_select[8:14]
    
    return ChangeVirtualHomeGraphSrvResponse(success=success)

if __name__ == "__main__":
    rospy.init_node('virtualhome_ros', anonymous=True)
    
    args = parse_args()
    prefab_classes, class_list = load_prefab_metadata("../resources/PrefabClass.json")
    
    comm = UnityCommunication()
    comm.timeout_wait = 300
    # with open(args.graph_path, "r") as f:
        # graph = json.load(f)
    # if graph is None:
        # raise ValueError(f"Failed to load graph from {args.graph_path}")
    
    # comm.reset()
    # success, message = comm.expand_scene(graph)
    # if not success:
        # print(f"Failed to load scene from {args.graph_path} to the simulator:", message)
        # sys.exit(1)
        
    rospy.Service('/moma/navigate', GetImageAtPoseSrv, handle_navigate_request)
    rospy.loginfo("Ready to navigate")
    rospy.Service('/moma/observe', GetImageSrv, handle_observe_request)
    rospy.loginfo("Ready to observe")
    rospy.Service('/moma/visible_objects', GetVisibleObjectsSrv, handle_visible_objects_request)
    rospy.loginfo("Ready to return visible objects")
    rospy.Service('/moma/find_object', FindObjectSrv, handle_find_request)
    rospy.loginfo("Ready to find objects")
    rospy.Service('/moma/pick_object', PickObjectSrv, handle_pick_request)
    rospy.loginfo("Ready to pick objects")
    rospy.Service('/moma/change_virtualhome_graph', ChangeVirtualHomeGraphSrv, handle_virtualhome_scene_request)
    rospy.loginfo("Ready to change virtual home graph")
    
    rospy.spin()