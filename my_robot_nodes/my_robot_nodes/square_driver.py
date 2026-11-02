#!/usr/bin/env python3
"""Drive four straight sides separated by quarter-turn action goals."""

import math

import rclpy
from action_msgs.msg import GoalStatus
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from lab_interfaces.action import DriveDistance
from my_robot_interfaces.action import Rotate


class SquareDriver(Node):

    def __init__(self):
        super().__init__("square_driver")
        self.declare_parameter("side_length", 0.5)
        self.odom_count = 0
        self.create_subscription(Odometry, "odom", self.on_odom, 10)
        self.drive_client = ActionClient(self, DriveDistance, "drive_distance")
        self.rotate_client = ActionClient(self, Rotate, "rotate")

    def on_odom(self, _msg):
        self.odom_count += 1

    def wait_until_ready(self):
        for client, name in (
            (self.drive_client, "/drive_distance"),
            (self.rotate_client, "/rotate"),
        ):
            self.get_logger().info(f"Waiting for {name}")
            while rclpy.ok() and not client.wait_for_server(timeout_sec=1.0):
                pass
            if not rclpy.ok():
                return False

        # Wait for fresh samples after both servers exist so their odometry callbacks
        # are initialized before the first goal arrives.
        required_odom_count = self.odom_count + 5
        self.get_logger().info("Waiting for odometry")
        while rclpy.ok() and self.odom_count < required_odom_count:
            rclpy.spin_once(self, timeout_sec=0.5)
        if not rclpy.ok():
            return False
        return True

    def run_goal(self, client, goal, description, side):
        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        if not send_future.done() or send_future.result() is None:
            self.get_logger().error(f"Side {side}: failed to send {description} goal")
            return False

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f"Side {side}: {description} goal was rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        if not result_future.done() or result_future.result() is None:
            self.get_logger().error(f"Side {side}: no result for {description} goal")
            return False

        status = result_future.result().status
        if status != GoalStatus.STATUS_SUCCEEDED:
            outcome = {
                GoalStatus.STATUS_ABORTED: "aborted",
                GoalStatus.STATUS_CANCELED: "canceled",
            }.get(status, f"ended with status {status}")
            self.get_logger().error(f"Side {side}: {description} goal {outcome}")
            return False
        return True

    def drive_square(self):
        side_length = self.get_parameter("side_length").value
        if not math.isfinite(side_length) or side_length <= 0.0:
            self.get_logger().error("side_length must be finite and greater than zero")
            return False
        if not self.wait_until_ready():
            return False

        for side in range(1, 5):
            self.get_logger().info(f"Driving side {side} of 4")
            drive_goal = DriveDistance.Goal()
            drive_goal.distance = float(side_length)
            if not self.run_goal(self.drive_client, drive_goal, "drive", side):
                return False

            rotate_goal = Rotate.Goal()
            rotate_goal.angle = float(math.pi / 2.0)
            if not self.run_goal(self.rotate_client, rotate_goal, "rotate", side):
                return False

        self.get_logger().info("Square complete")
        return True


def main():
    rclpy.init()
    node = SquareDriver()
    try:
        node.drive_square()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
