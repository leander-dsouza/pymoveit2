#!/usr/bin/env python3
"""
Example of moving to a joint configuration.
`ros2 run pymoveit2 ex_joint_goal.py --ros-args -p joint_positions:="[0.0, -1.57, 0.0, 0.0, 0.0, 0.0]"`
"""

from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2 import MoveIt2
from pymoveit2.robots import ur5e

import ikpy.chain

my_chain = ikpy.chain.Chain.from_urdf_file("ur5e.urdf")


def main(args=None):

    rclpy.init(args=args)

    # Create node for this example
    node = Node("ex_joint_goal")

    # Target position
    target_position = [1.0, -1.0, 1.0]

    # Target joint angles
    target_joint_angles = my_chain.inverse_kinematics(target_position)

    target_joint_angles = target_joint_angles.tolist()[1:6]


    # Declare parameter for joint positions

    node.declare_parameter(
        "joint_positions", target_joint_angles
    )

    # node.declare_parameter(
    #     "joint_positions",
    #     [
    #         0.0,
    #         0.0,
    #         0.0,
    #         0.0,
    #         0.0,
    #         0.0,
    #     ],
    # )

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

    # Get parameter
    joint_positions = (
        node.get_parameter("joint_positions").get_parameter_value().double_array_value
    )

    # Move to joint configuration
    node.get_logger().info(f"Moving to {{joint_positions: {list(joint_positions)}}}")
    moveit2.move_to_configuration(joint_positions)
    moveit2.wait_until_executed()

    rclpy.shutdown()
    exit(0)


if __name__ == "__main__":
    main()
