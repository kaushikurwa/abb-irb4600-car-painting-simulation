import rclpy

from rclpy.node import Node
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK


class RailIKTest(Node):

    def __init__(self):
        super().__init__("rail_ik_test")

        self.js = None

        self.sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.state_callback,
            10
        )

        self.client = self.create_client(
            GetPositionIK,
            "/compute_ik"
        )

    def state_callback(self, msg):
        self.js = msg


def main():

    rclpy.init()
    node = RailIKTest()

    print("Waiting for IK service...")

    while not node.client.wait_for_service(timeout_sec=1.0):
        pass

    print("Waiting for joint states...")

    while node.js is None:
        rclpy.spin_once(node, timeout_sec=0.1)

    target = [-1.2, 0.41, 1.63437]

    # Scan the complete rail range in 0.2 m increments.
    rail_positions = [
        round(-3.2 + i * 0.2, 2)
        for i in range(33)
    ]

    print("\nFar-side rail IK diagnostic")
    print("Target:", target)
    print("Orientation: original spray orientation")
    print("Collision checking: disabled for diagnosis")
    print("\nResults:")

    successful_positions = []

    for rail_position in rail_positions:

        start_state = JointState()

        start_state.header = node.js.header
        start_state.name = list(node.js.name)
        start_state.position = list(node.js.position)

        if "rail_joint" not in start_state.name:
            print("ERROR: rail_joint not found.")
            break

        rail_index = start_state.name.index("rail_joint")
        start_state.position[rail_index] = rail_position

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

        future = node.client.call_async(req)
        rclpy.spin_until_future_complete(node, future)

        result = future.result()

        if result is None:
            print(
                f"Rail {rail_position:+.2f} m: "
                "No service result"
            )
            continue

        error = result.error_code.val

        if error == 1:
            solution = result.solution.joint_state

            arm_solution = {}

            for name, position in zip(
                solution.name,
                solution.position
            ):
                if name.startswith("joint_"):
                    arm_solution[name] = round(position, 3)

            successful_positions.append(rail_position)

            print(
                f"Rail {rail_position:+.2f} m: SUCCESS | "
                f"Arm: {arm_solution}"
            )

        else:
            print(
                f"Rail {rail_position:+.2f} m: "
                f"IK error {error}"
            )

    print("\n========== SUMMARY ==========")

    if successful_positions:
        print("Reachable rail positions:")
        print(successful_positions)
    else:
        print("No reachable rail positions found.")

    print("=============================")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()