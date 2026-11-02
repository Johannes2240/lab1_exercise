#!/usr/bin/env python3
"""Action server that rotates the robot through an angle measured from odometry."""

import math
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

from my_robot_interfaces.action import Rotate


def yaw_from_quaternion(quaternion):
    """Return the yaw component of a quaternion in radians."""
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def angle_difference(current, previous):
    """Return current - previous wrapped to [-pi, pi]."""
    return math.atan2(math.sin(current - previous), math.cos(current - previous))


class RotateServer(Node):

    def __init__(self):
        super().__init__("rotate_server")
        self.declare_parameter("turn_speed", 0.5)
        self.yaw = None
        self.execution_lock = threading.Lock()

        callback_group = ReentrantCallbackGroup()
        self.cmd_vel = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(
            Odometry,
            "odom",
            self.on_odom,
            10,
            callback_group=callback_group,
        )
        self.action_server = ActionServer(
            self,
            Rotate,
            "rotate",
            execute_callback=self.execute,
            goal_callback=self.on_goal,
            cancel_callback=lambda goal_handle: CancelResponse.ACCEPT,
            callback_group=callback_group,
        )

    def on_odom(self, msg):
        self.yaw = yaw_from_quaternion(msg.pose.pose.orientation)

    def on_goal(self, goal):
        if not math.isfinite(goal.angle) or goal.angle == 0.0:
            self.get_logger().warning("Rejected a zero or non-finite rotation goal")
            return GoalResponse.REJECT
        if self.yaw is None:
            self.get_logger().warning("Rejected rotation goal because odometry is not ready")
            return GoalResponse.REJECT
        if self.execution_lock.locked():
            self.get_logger().warning("Rejected rotation goal because another goal is active")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def execute(self, goal_handle):
        target = goal_handle.request.angle
        direction = 1.0 if target > 0.0 else -1.0
        speed = abs(self.get_parameter("turn_speed").value)
        result = Rotate.Result()

        if not self.execution_lock.acquire(blocking=False):
            self.get_logger().warning("Aborted overlapping rotation goal")
            goal_handle.abort()
            return result

        try:
            if not math.isfinite(speed) or speed == 0.0:
                self.get_logger().error("turn_speed must be finite and greater than zero")
                goal_handle.abort()
                return result

            previous_yaw = self.yaw
            turned = 0.0
            command = Twist()
            command.angular.z = direction * speed
            self.get_logger().info(f"Rotating {target:.2f} rad")

            while rclpy.ok():
                current_yaw = self.yaw
                turned += angle_difference(current_yaw, previous_yaw)
                previous_yaw = current_yaw
                result.turned = float(turned)

                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.get_logger().info(f"Canceled after {turned:.2f} rad")
                    return result

                if direction * turned >= abs(target):
                    goal_handle.succeed()
                    self.get_logger().info(f"Rotation complete at {turned:.2f} rad")
                    return result

                self.cmd_vel.publish(command)
                feedback = Rotate.Feedback()
                feedback.turned = float(turned)
                goal_handle.publish_feedback(feedback)
                time.sleep(0.05)

            return result
        finally:
            if rclpy.ok():
                self.cmd_vel.publish(Twist())
            self.execution_lock.release()


def main():
    rclpy.init()
    node = RotateServer()
    try:
        rclpy.spin(node, executor=MultiThreadedExecutor())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
