#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


def main():
    rclpy.init()
    node = Node("open_left_hand")
    client = ActionClient(
        node,
        FollowJointTrajectory,
        "/left_hand_controller/follow_joint_trajectory",
    )

    try:
        node.get_logger().info("Waiting for left hand controller ...")
        if not client.wait_for_server(timeout_sec=10.0):
            node.get_logger().error("Left hand controller not found")
            return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [
            "L_thumb_proximal_yaw_joint",
            "L_thumb_proximal_pitch_joint",
            "L_index_proximal_joint",
            "L_middle_proximal_joint",
            "L_ring_proximal_joint",
            "L_pinky_proximal_joint",
        ]

        point = JointTrajectoryPoint()
        point.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        point.time_from_start = Duration(seconds=2.0).to_msg()
        goal.trajectory.points = [point]

        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, future)
        handle = future.result()

        if not handle.accepted:
            node.get_logger().error("Goal rejected")
            return

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future)
        result = result_future.result().result

        if result.error_code == 0:
            node.get_logger().info("Left hand opened")
        else:
            node.get_logger().error(
                f"Failed: {result.error_code} {result.error_string}"
            )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
