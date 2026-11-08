#!/usr/bin/env python3
"""Add or remove a box obstacle in the MoveIt planning scene.  Usage: scene_box.py add | remove"""
import sys

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive

NAME = "obstacle_box"
CENTER = (-0.45, -0.30, 1.75)     # world x, y, z of the box centre (m): above the roof, across stripe roof_03
SIZE = (0.30, 0.30, 0.30)         # box size x, y, z (m)


def main():
    action = sys.argv[1] if len(sys.argv) > 1 else "add"
    rclpy.init()
    node = rclpy.create_node("scene_box")
    cli = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
    if not cli.wait_for_service(timeout_sec=10.0):
        print("/apply_planning_scene not available - is the MoveIt demo running?")
        return
    obj = CollisionObject()
    obj.header.frame_id = "world"
    obj.id = NAME
    if action == "add":
        prim = SolidPrimitive()
        prim.type = SolidPrimitive.BOX
        prim.dimensions = list(SIZE)
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = CENTER
        pose.orientation.w = 1.0
        obj.primitives.append(prim)
        obj.primitive_poses.append(pose)
        obj.operation = CollisionObject.ADD
    else:
        obj.operation = CollisionObject.REMOVE
    scene = PlanningScene()
    scene.is_diff = True
    scene.world.collision_objects.append(obj)
    req = ApplyPlanningScene.Request()
    req.scene = scene
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut)
    print("box %s: %s" % (action, "ok" if fut.result().success else "FAILED"))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
