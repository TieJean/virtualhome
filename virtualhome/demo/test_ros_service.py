## ROS Service Calls
import rospy
import roslib; roslib.load_manifest('amrl_msgs')
from amrl_msgs.srv import (
    GetImageSrv,
    GetImageAtPoseSrv, 
    GetImageAtPoseSrvRequest, 
    PickObjectSrv, 
    PickObjectSrvRequest,
    DetectVirtualHomeObjectSrv,
    DetectVirtualHomeObjectSrvRequest,
    ChangeVirtualHomeGraphSrv,
    ChangeVirtualHomeGraphSrvRequest,
    OpenVirtualHomeObjectSrv,
    OpenVirtualHomeObjectSrvRequest,
)
from ros_utils import *

def test_observe_request():
    rospy.wait_for_service("/moma/observe")
    try:
        observe_service = rospy.ServiceProxy("/moma/observe", GetImageSrv)
        response = observe_service()
    except rospy.ServiceException as e:
        print("Service call failed:", e)
    pil_image = ros_image_to_pil(response.image)        
    pil_image.save("../../outputs/debug_observe.png")
    
def test_navigate_request():
    rospy.wait_for_service("/moma/navigate")
    try:
        navigate_service = rospy.ServiceProxy("/moma/navigate", GetImageAtPoseSrv)
        request = GetImageAtPoseSrvRequest()
        request.x = 5.816992
        request.y = 0
        request.z = 0.5
        response = navigate_service(request)
    except rospy.ServiceException as e:
        print("Service call failed:", e)
    print("Navigate success:", response.success)
    
def test_detect_virtualhome_request():
    rospy.wait_for_service("/moma/detect_virtual_home_object")
    try:
        detect_service = rospy.ServiceProxy("/moma/detect_virtual_home_object", DetectVirtualHomeObjectSrv)
        request = DetectVirtualHomeObjectSrvRequest()
        request.query_text = "folder"
        response = detect_service(request)
    except rospy.ServiceException as e:
        print("Service call failed:", e)
    if response.success:
        for instance_id in response.ids:
            print("Detected instance ID:", instance_id)
            
def test_virtualhome_scene_change():
    rospy.wait_for_service("/moma/change_virtualhome_graph")
    try:
        scene_change_service = rospy.ServiceProxy("/moma/change_virtualhome_graph", ChangeVirtualHomeGraphSrv)
        request = ChangeVirtualHomeGraphSrvRequest()
        request.scene_id = 4
        request.graph_path = "../../unity_output/scene4_00_interactive/0/graph.json"
        response = scene_change_service(request)
    except rospy.ServiceException as e:
        print("Service call failed:", e)
        
def test_open_virtualhome_request():
    rospy.wait_for_service("/moma/open_object")
    try:
        open_service = rospy.ServiceProxy("/moma/open_object", OpenVirtualHomeObjectSrv)
        request = OpenVirtualHomeObjectSrvRequest()
        request.instance_id = 143
        response = open_service(request)
    except rospy.ServiceException as e:
        print("Service call failed:", e)
    print("Open object success:", response.success)
        
def test_pick_object_request():
    rospy.wait_for_service("/moma/pick_object")
    try:
        pick_service = rospy.ServiceProxy("/moma/pick_object", PickObjectSrv)
        request = PickObjectSrvRequest()
        request.instance_id = 370
        response = pick_service(request)
    except rospy.ServiceException as e:
        print("Service call failed:", e)
    print("Pick object success:", response.success)

if __name__ == "__main__":
    rospy.init_node('virtualhome_ros_client', anonymous=True)
    test_virtualhome_scene_change()
    test_navigate_request()
    # test_detect_virtualhome_request()
    test_open_virtualhome_request()
    test_detect_virtualhome_request()
    test_pick_object_request()