import json
import rclpy
import os

from rclpy.node import Node
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetCartesianPath, GetPositionIK


class CartesianTest(Node):

    def __init__(self):
        super().__init__("far_cartesian_test")

        self.js = None

        self.sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.state_callback,
            10
        )

        self.client = self.create_client(
            GetCartesianPath,
            "/compute_cartesian_path"
        )

        self.ik_client = self.create_client(
            GetPositionIK,
            "/compute_ik"
        )

    def state_callback(self, msg):
        self.js = msg


def main():

    rclpy.init()
    node = CartesianTest()

    print("Waiting for Cartesian path service...")

    while not node.client.wait_for_service(timeout_sec=1.0):
        pass

    print("Waiting for IK service...")

    while not node.ik_client.wait_for_service(timeout_sec=1.0):
        pass

    print("Waiting for joint states...")

    while node.js is None:
        rclpy.spin_once(node, timeout_sec=0.1)

    tools_dir = os.path.dirname(os.path.abspath(__file__))
    file_path = os.path.join(tools_dir, "roof_far_stripes.json")

    with open(file_path) as f:
        stripes = json.load(f)

    stripe = next(
        s for s in stripes if s["name"] == "far_00"
    )

    points = stripe["points"]

    def make_pose(pt):

        pose = Pose()

        pose.position.x = pt[0]
        pose.position.y = pt[1]
        pose.position.z = pt[2]

        pose.orientation.x = 1.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 0.0

        return pose

    def compute_path(start_state, waypoints):

        req = GetCartesianPath.Request()

        req.header.frame_id = "world"
        req.start_state.joint_state = start_state
        req.group_name = "arm"
        req.link_name = "paint_tcp"
        req.waypoints = [make_pose(p) for p in waypoints]
        req.max_step = 0.005
        req.jump_threshold = 0.0
        req.avoid_collisions = False

        future = node.client.call_async(req)

        rclpy.spin_until_future_complete(node, future)

        return future.result()

    print("\nReaching waypoint 5.")

    current_state = node.js

    for i, pt in enumerate(points[1:5], start=2):

        result = compute_path(current_state, [pt])

        if result is None:
            print("Cartesian service returned no result.")
            node.destroy_node()
            rclpy.shutdown()
            return

        trajectory = result.solution.joint_trajectory.points

        print(
            "Waypoint %d: %.1f%% feasible"
            % (i, result.fraction * 100)
        )

        if result.fraction < 0.999 or len(trajectory) < 2:

            print("Failed before waypoint 5.")
            print("MoveIt error:", result.error_code.val)

            node.destroy_node()
            rclpy.shutdown()
            return

        current_state = JointState()

        current_state.name = list(
            result.solution.joint_trajectory.joint_names
        )

        current_state.position = list(
            trajectory[-1].positions
        )
        print("\nWaypoint 5 joint configuration:")

        for name, value in zip(
            current_state.name,
            current_state.position
        ):
            print("%s: %.4f rad (%.2f deg)" % (
                name, value, value * 57.2958
            ))

    target = [-1.2, 0.41, 1.63437]

    print("\nTesting multiple IK seed configurations.")

    import math

    seed_configs = [
        ("Waypoint 5", [0.0, 1.1953, -1.4037, 0.0, 1.7792, -1.5708]),
        ("Home", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        ("Seed 1", [0.0, 0.5, -0.5, 0.0, 1.0, -1.57]),
        ("Seed 2", [0.0, 0.8, -1.0, 0.0, 1.5, -1.57]),
        ("Seed 3", [0.5, 0.5, -0.5, 0.5, 1.0, -1.0]),
        ("Seed 4", [-0.5, 0.5, -0.5, -0.5, 1.0, -2.0])
    ]

    arm_joint_names = [
        "joint_1",
        "joint_2",
        "joint_3",
        "joint_4",
        "joint_5",
        "joint_6"
    ]

    for label, seed in seed_configs:

        start_state = JointState()

        start_state.name = list(node.js.name)
        start_state.position = list(node.js.position)

        for joint_name, value in zip(arm_joint_names, seed):

            index = start_state.name.index(joint_name)
            start_state.position[index] = value

        req = GetPositionIK.Request()
        ik = req.ik_request

        ik.group_name = "arm"
        ik.ik_link_name = "paint_tcp"
        ik.avoid_collisions = False
        ik.timeout = Duration(sec=2, nanosec=0)

        ik.robot_state.joint_state = start_state
        ik.robot_state.is_diff = False

        ps = PoseStamped()
        ps.header.frame_id = "world"

        ps.pose.position.x = target[0]
        ps.pose.position.y = target[1]
        ps.pose.position.z = target[2]

        ps.pose.orientation.x = 1.0
        ps.pose.orientation.y = 0.0
        ps.pose.orientation.z = 0.0
        ps.pose.orientation.w = 0.0

        ik.pose_stamped = ps

        future = node.ik_client.call_async(req)
        rclpy.spin_until_future_complete(node, future)

        result = future.result()

        if result is None:
            print(label, "| No service result")
            continue

        print(
            label,
            "| IK error:",
            result.error_code.val
        )

    print("\nMultiple-seed IK test completed.")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
