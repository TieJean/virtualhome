## ROS Service Calls
import rospy
import roslib; roslib.load_manifest('amrl_msgs')
from amrl_msgs.srv import (
    GetImageSrv,
    GetImageAtPoseSrv, 
    GetImageAtPoseSrvRequest, 
    PickObjectSrv, 
    PickObjectSrvResponse
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
        request.x = 1.078848
        request.y = 1.005129
        request.z = 0.7974954
        response = navigate_service(request)
    except rospy.ServiceException as e:
        print("Service call failed:", e)
    print("Navigate success:", response.success)
    if response.image.data:
        pil_image = ros_image_to_pil(response.image)        
        pil_image.save("../../outputs/debug_navigate.png")

if __name__ == "__main__":
    rospy.init_node('virtualhome_ros_client', anonymous=True)
    test_observe_request()
    test_navigate_request()