import copy
import json
import rclpy
import os

from rclpy.node import Node
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState


class IKTest(Node):

    def __init__(self):
        super().__init__("far_ik_test")

        self.client = self.create_client(
            GetPositionIK, "/compute_ik"
        )

        self.js = None
        self.sub = self.create_subscription(
            JointState, "/joint_states",
            self.state_callback, 10
        )

    def state_callback(self, msg):
        self.js = msg


def main():
    rclpy.init()
    node = IKTest()

    while not node.client.wait_for_service(timeout_sec=1.0):
        print("Waiting for IK service...")

    while node.js is None:
        rclpy.spin_once(node, timeout_sec=0.1)

tools_dir = os.path.dirname(os.path.abspath(__file__))

with open(os.path.join(tools_dir, "roof_far_stripes.json")) as f:
        stripes = json.load(f)

    test_points = [
        ("near_hover", [-1.2, -0.55, 1.765]),
        ("far_hover", [-1.2, 0.0, 1.813])
    ]

    for label, pt in test_points:
        req = GetPositionIK.Request()
        ik = req.ik_request

        ik.group_name = "arm"
        ik.ik_link_name = "paint_tcp"
        ik.avoid_collisions = True
        ik.timeout = Duration(sec=0, nanosec=300_000_000)

        ik.robot_state.joint_state = node.js
        ik.robot_state.is_diff = False

        ps = PoseStamped()
        ps.header.frame_id = "world"
        ps.pose.position.x = pt[0]
        ps.pose.position.y = pt[1]
        ps.pose.position.z = pt[2]

        ps.pose.orientation.x = 1.0
        ps.pose.orientation.y = 0.0
        ps.pose.orientation.z = 0.0
        ps.pose.orientation.w = 0.0

        ik.pose_stamped = ps

        future = node.client.call_async(req)
        rclpy.spin_until_future_complete(node, future)

        result = future.result()

        print(label, "Point:", pt, "IK error:", result.error_code.val)

    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
