import pathlib

import rclpy
from rclpy.node import Node

from amrl_msgs.srv import (
    ChangeVirtualHomeGraphSrv,
    DetectVirtualHomeObjectSrv,
    GetImageAtPoseSrv,
    GetImageSrv,
    OpenVirtualHomeObjectSrv,
    PickObjectSrv,
)

from ros_utils import ros_image_to_pil


GetImageAtPoseSrvRequest = GetImageAtPoseSrv.Request
PickObjectSrvRequest = PickObjectSrv.Request
DetectVirtualHomeObjectSrvRequest = DetectVirtualHomeObjectSrv.Request
ChangeVirtualHomeGraphSrvRequest = ChangeVirtualHomeGraphSrv.Request
OpenVirtualHomeObjectSrvRequest = OpenVirtualHomeObjectSrv.Request


class VirtualHomeRos2Client(Node):
    def __init__(self):
        super().__init__("virtualhome_ros2_client")

    def call_service(self, service_name, srv_type, request=None, timeout_sec=30.0):
        client = self.create_client(srv_type, service_name)
        if not client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error(f"Service unavailable: {service_name}")
            return None

        if request is None:
            request = srv_type.Request()

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if not future.done():
            self.get_logger().error(f"Timed out calling service: {service_name}")
            return None

        if future.exception() is not None:
            self.get_logger().error(f"Service call failed ({service_name}): {future.exception()}")
            return None

        return future.result()

    def test_observe_request(self):
        response = self.call_service("/moma/observe", GetImageSrv)
        if response is None:
            return

        pil_image = ros_image_to_pil(response.image)
        output_path = pathlib.Path(__file__).resolve().parents[2] / "outputs" / "debug_observe.png"
        pil_image.save(output_path)
        self.get_logger().info(f"Saved observe image to: {output_path}")

    def test_navigate_request(self):
        request = GetImageAtPoseSrvRequest()
        request.x = 5.816992
        request.y = 0.0
        request.z = 0.5

        response = self.call_service("/moma/navigate", GetImageAtPoseSrv, request)
        if response is None:
            return
        print("Navigate success:", response.success)

    def test_detect_virtualhome_request(self):
        request = DetectVirtualHomeObjectSrvRequest()
        request.query_text = "folder"

        response = self.call_service(
            "/moma/detect_virtual_home_object",
            DetectVirtualHomeObjectSrv,
            request,
        )
        if response is None:
            return

        if response.success:
            for instance_id in response.ids:
                print("Detected instance ID:", instance_id)

    def test_virtualhome_scene_change(self):
        request = ChangeVirtualHomeGraphSrvRequest()
        request.scene_id = 4
        request.graph_path = "../../unity_output/scene4_00_interactive/0/graph.json"

        response = self.call_service("/moma/change_virtualhome_graph", ChangeVirtualHomeGraphSrv, request)
        if response is None:
            return
        print("Scene change success:", response.success)

    def test_open_virtualhome_request(self):
        request = OpenVirtualHomeObjectSrvRequest()
        request.instance_id = 143

        response = self.call_service("/moma/open_object", OpenVirtualHomeObjectSrv, request)
        if response is None:
            return
        print("Open object success:", response.success)

    def test_pick_object_request(self):
        request = PickObjectSrvRequest()
        request.instance_id = 370

        response = self.call_service("/moma/pick_object", PickObjectSrv, request)
        if response is None:
            return
        print("Pick object success:", response.success)


def main():
    rclpy.init()
    node = VirtualHomeRos2Client()

    try:
        node.test_virtualhome_scene_change()
        node.test_navigate_request()
        node.test_open_virtualhome_request()
        node.test_detect_virtualhome_request()
        node.test_pick_object_request()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()