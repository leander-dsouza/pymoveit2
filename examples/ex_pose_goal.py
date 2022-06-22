#!/usr/bin/env python3
"""
Example of moving to a pose goal.
`ros2 run pymoveit2 ex_pose_goal.py --ros-args -p position:="[0.25, 0.0, 1.0]" -p quat_xyzw:="[0.0, 0.0, 0.0, 1.0]"`
"""

from math import degrees, radians
from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2 import MoveIt2
from pymoveit2.robots import ur5e


def main(args=None):

    rclpy.init(args=args)

    # Create node for this example
    node = Node("ex_pose_goal")

    # Declare parameters for position and orientation
    node.declare_parameter("position", [0.5, 0.0, 0.25])
    node.declare_parameter("quat_xyzw", [1.0, 0.0, 0.0, 0.0])

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()

    # Create MoveIt 2 interface
    moveit2 = MoveIt2(
        node=node,
        joint_names=ur5e.joint_names(),
        base_link_name=ur5e.base_link_name(),
        end_effector_name=ur5e.end_effector_name(),
        group_name=ur5e.MOVE_GROUP_ARM,
        callback_group=callback_group,
    )

    # Spin the node in background thread(s)
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()

    # Get parameters
    position = node.get_parameter("position").get_parameter_value().double_array_value
    quat_xyzw = node.get_parameter("quat_xyzw").get_parameter_value().double_array_value

    # Move to pose
    node.get_logger().info(
        f"Moving to {{position: {list(position)}, quat_xyzw: {list(quat_xyzw)}}}"
    )

    moveit2.move_to_pose(
        position=position,
        quat_xyzw=quat_xyzw,
        path_joint_constraints=[
            (ur5e.joint_names()[1], radians(-90), radians(-45), radians(45)),
            (ur5e.joint_names()[2], radians(0), radians(-1), radians(170)),
            (ur5e.joint_names()[3], radians(-90), radians(-90), radians(90)),
        ],
    )
    moveit2.wait_until_executed()
    position[2] -= 0.1
    print("--" * 5)
    moveit2.move_to_pose(
        position=position,
        quat_xyzw=quat_xyzw,
        path_joint_constraints=[
            (ur5e.joint_names()[1], radians(-90), radians(-45), radians(45)),
            (ur5e.joint_names()[2], radians(0), radians(-1), radians(170)),
            (ur5e.joint_names()[3], radians(-90), radians(-90), radians(90)),
        ],
        cartesian=True,
    )
    position[1] -= 0.3
    moveit2.wait_until_executed()
    print("--" * 5)
    moveit2.move_to_pose(
        position=position,
        quat_xyzw=quat_xyzw,
        path_joint_constraints=[
            (ur5e.joint_names()[1], radians(-90), radians(-45), radians(45)),
            (ur5e.joint_names()[2], radians(0), radians(-1), radians(170)),
            (ur5e.joint_names()[3], radians(-90), radians(-90), radians(90)),
        ],
        cartesian=True,
    )
    moveit2.wait_until_executed()

    rclpy.shutdown()
    exit(0)


if __name__ == "__main__":
    main()
