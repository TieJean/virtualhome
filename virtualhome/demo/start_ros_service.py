import argparse
import json
import sys
from PIL import ImageDraw

from langchain_openai import ChatOpenAI
from langchain_core.messages import HumanMessage, SystemMessage
from langchain_core.prompts import ChatPromptTemplate

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
from geometry_msgs.msg import Point

comm = None
class_list = None
cameras_select = None
pano_camera_select = None
vlm = None

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
    if ok_img:
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
    if not success:
        import pdb; pdb.set_trace()
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

def _get_query_text(txt: str) -> str:
    if "toy" in txt or "action figure" in txt or "transformer" in txt or "robot" in txt or "plush" in txt or "animal" in txt or "teddy" in txt or "train" in txt:
        return "toy"
    elif "book" in txt or "biography" in txt or "novel" in txt:
        return "book"
    elif "folder" in txt or "binder" in txt or "doc" in txt:
        return "folder"
    elif "magazine" in txt or "issue" in txt or "mag" in txt:
        return "magazine"
    else:
        raise ValueError(f"Unknown query text: {txt}")
    
def _find_instance(query_text: str, query_cls: str, ref_image):
    """
    Find the instance UID of the object based on the query text.
    """
    global comm, vlm
    
    messages = []
    messages += [
        SystemMessage(content=(
           "You are a visual object-matching assistant. "
            "The user is looking for a specific object and will provide (1) a text description and (2) a reference image of the object as previously observed. "
            "Next, you will be shown several current camera views. Each view contains **red bounding boxes** labeled `Instance: {i}`. "
            "Your job is to determine whether any of the labeled instances match the reference object. "
            "If a match exists, reply with the **single most confident** instance ID (e.g., 0, 1, 2, ...). "
            "If no match is present, reply with **-1**. "
            "**Do not explain or justify your choice — reply with the integer only.**"
        ))
    ]
    messages += [
        HumanMessage(content=(
            f"The user is searching for: {query_text}. "
            "If any of the candidate instances match this object, reply with the matching instance ID. "
            "If none of them match, reply with -1. "
            "This is the reference image showing where the user last saw the object:"
        ))
    ]
    
    # Step 1: Encode reference image
    encoded_img = ros_image_to_base64(ref_image)
    ref_img_msg = [get_vlm_img_message(encoded_img)]
    
    messages += [HumanMessage(content=ref_img_msg)]
    
    # Step 2: Get images from simulator
    (ok_img, imgs) = comm.camera_image(pano_camera_select, mode="normal")
    (ok_img, cls_imgs) = comm.camera_image(pano_camera_select, mode="seg_class")
    (ok_img, inst_imgs) = comm.camera_image(pano_camera_select, mode="seg_inst")

    # Step 3: Save for debug
    view_pil = display_grid_img(imgs + cls_imgs + inst_imgs, nrows=3)
    view_pil.save("../../outputs/debug_find_instance.png")

    # Step 4: Get scene graph
    success, graph = comm.environment_graph()
    
    # Step 5: Parse object color map 
    success, instance_colors = comm.instance_colors()
    
    # Step 6: Find IDs of all matching the target class objects
    target_ids = []
    for node in graph["nodes"]:
        if query_cls.lower() == node.get("class_name", "").lower():
            target_ids.append(str(node["id"]))
            
    # Step 7: Convert instance colors to uint8
    target_bgr_colors = []
    for uid in target_ids:
        rgb = instance_colors.get(uid)
        if rgb:
            bgr_uint8 = bgr_uint8 = (
                int(round(rgb[2] * 255)),  # B
                int(round(rgb[1] * 255)),  # G
                int(round(rgb[0] * 255))   # R
            ) 
            target_bgr_colors.append(bgr_uint8)

    print("Target instance IDs:", target_ids)
    print("Target colors:", target_bgr_colors)
    
    messages += [
        HumanMessage(content=(
            "Now, you will be shown several camera views: "
        ))
    ]
    
    # Step 8: Iterate over inst_imgs and draw boxes
    any_box_drawn = False  # <-- Add this
    for i, (rgb_img, inst_img) in enumerate(zip(imgs, inst_imgs)):
        img_vis = rgb_img.copy()

        for uid, color in zip(target_ids, target_bgr_colors):
            mask = cv2.inRange(inst_img, np.array(color), np.array(color))  # exact match
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if contours:
                any_box_drawn = True  # <-- Set if any contour found
                
            for cnt in contours:
                x, y, w, h = cv2.boundingRect(cnt)

                # Draw bounding box
                cv2.rectangle(img_vis, (x, y), (x + w, y + h), (0, 0, 255), 2)

                # Add label using node ID
                label = f"Instance ID: {uid}"
                cv2.putText(
                    img_vis,
                    label,
                    (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 0, 255),
                    1,
                    cv2.LINE_AA
                )
        
        encoded_img = opencv_image_to_base64(img_vis)
        encoded_img = [get_vlm_img_message(encoded_img)]
        messages += [HumanMessage(content=encoded_img)]
        cv2.imwrite(f"../../outputs/seg_debug_view_{i}.png", img_vis)
        
    # Early return if no bounding boxes were drawn
    if not any_box_drawn:
        return None
        
    chat_prompt = ChatPromptTemplate.from_messages(messages)
    chained_model = chat_prompt | vlm
    
    instance_id = None
    for attempt in range(2):
        response = chained_model.invoke({})
        try:
            instance_id = int(response.content.strip())
            break  # Success
        except Exception as e:
            if attempt == 1:
                raise ValueError(f"Invalid response from model: {response.content}") from e
    
    if instance_id == -1:
        instance_id = None
    
    return instance_id

def _get_visible_instances():
    (ok_img, imgs) = comm.camera_image(pano_camera_select, mode="normal")
    (ok_img, cls_imgs) = comm.camera_image(pano_camera_select, mode="seg_class")
    (ok_img, inst_imgs) = comm.camera_image(pano_camera_select, mode="seg_inst")
    
    # Step 3: Save for debug
    view_pil = display_grid_img(imgs + cls_imgs + inst_imgs, nrows=3)
    view_pil.save("../../outputs/debug_get_visible_instances.png")
    
    success, graph = comm.environment_graph()
    success, instance_colors = comm.instance_colors()
    
    assert len(imgs) == len(cls_imgs) == len(inst_imgs), "Number of images mismatch"
    
    frame_nodes = set()
    for img, cls_img, inst_img in zip(imgs, cls_imgs, inst_imgs):
        unique_inst_colors = np.unique(inst_img.reshape(-1, 3), axis=0)
        for inst_color in unique_inst_colors:
            if np.all(inst_color == 0):
                continue  # skip background
            mask_inst = np.all(inst_img == inst_color, axis=-1)
            if np.sum(mask_inst) < 10:
                continue
            class_colors, counts = np.unique(cls_img[mask_inst].reshape(-1, 3), axis=0, return_counts=True)
            class_color = class_colors[np.argmax(counts)]
            
            matched_node = None
            for node in graph["nodes"]:
                node_id = str(node["id"])
                prefab_name = node.get("prefab_name", "")
                rgb_f = instance_colors.get(node_id)
                if rgb_f is None:
                    continue
                
                node_inst_color = np.array([rgb_f[2], rgb_f[1], rgb_f[0]]) * 255
                node_inst_color = node_inst_color.astype(np.uint8)
                
                if not np.allclose(inst_color, node_inst_color, atol=2):
                    continue
                
                try:
                    node_class_color = semantic_cls_to_bgr(node["class_name"], class_list)
                except ValueError:
                    continue
                if not np.allclose(class_color, node_class_color, atol=2):
                    continue
                matched_node = node
                break
            
            if matched_node is not None:
                frame_nodes.add(matched_node["prefab_name"])
    return frame_nodes
    
def handle_find_request(req):
    global comm
    rospy.loginfo("Received find request")
    
    find_success = False
    target_node_id = None
    target_position = None
    
    query_cls = _get_query_text(req.query_text.lower())
    if req.ref_image:
        target_node_id = _find_instance(req.query_text, query_cls, req.ref_image)
    else:
        target_node_id = find_target_node_id(query_cls)
        
    visible_instances = _get_visible_instances()
    # import pdb; pdb.set_trace()
    
    success, graph = comm.environment_graph()
    
    if target_node_id is None:
        (ok_img, imgs) = comm.camera_image(pano_camera_select, mode="normal")
        view_pil = display_grid_img(imgs, nrows=2)
        view_pil.save("../../outputs/debug_find.png")
        rospy.logwarn(f"Object '{query_cls}' not found in visible objects.")
        return FindObjectSrvResponse(success=False)
        
    find_success = target_node_id is not None
    target_node = extract_nodes_by_ids(graph["nodes"], [target_node_id])
    if target_node is None or len(target_node) == 0:
        return FindObjectSrvResponse(success=False)
    position = target_node[0]["obj_transform"]["position"]
    target_position = Point(position[0], position[1], position[2])
    
    return FindObjectSrvResponse(
        success=find_success,
        id=target_node_id,
        position=target_position,
        visible_instances=list(visible_instances),
    )
    
def handle_pick_request(req):
    global comm
    rospy.loginfo("Received pick request")
    
    target_node_id = None
    query_text = _get_query_text(req.query_text.lower())
    if req.instance_id is not None:
        _, graph = comm.environment_graph()
        target_node_id = int(req.instance_id)
        target_node = extract_nodes_by_ids(graph["nodes"], [target_node_id])
        if len(target_node) != 1:
            return PickObjectSrvResponse(
                success=False,
            )
        target_node = target_node[0]
        if target_node["class_name"].lower() != query_text.lower():
            return PickObjectSrvResponse(
                success=False,
            )
    else:
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
    
    vlm = ChatOpenAI(model="o3", temperature=1, api_key=os.environ.get("OPENAI_API_KEY"))
        
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