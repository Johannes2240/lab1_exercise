#!/usr/bin/env python3
"""Drive forward and turn away from nearby obstacles while enabled."""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_srvs.srv import SetBool

from lab_interfaces.msg import ObstacleInfo
from my_robot_interfaces.msg import RobotStatus
from my_robot_interfaces.srv import SetTurnSpeed


class TurnAway(Node):

    def __init__(self):
        super().__init__("turn_away")
        self.declare_parameter("forward_speed", 0.2)
        self.declare_parameter("turn_speed", 0.5)
        self.declare_parameter("max_turn_speed", 1.5)

        self.enabled = False
        self.cmd_vel = self.create_publisher(Twist, "cmd_vel", 10)
        self.status = self.create_publisher(RobotStatus, "robot_status", 10)
        self.create_subscription(ObstacleInfo, "obstacle", self.on_obstacle, 10)
        self.create_service(SetBool, "turn_away_enable", self.on_enable)
        self.create_service(SetTurnSpeed, "set_turn_speed", self.on_set_turn_speed)

    def on_obstacle(self, msg):
        status = RobotStatus()
        status.obstacle_distance = float(msg.distance)

        if not self.enabled:
            status.state = "stopped"
        else:
            command = Twist()
            if msg.too_close:
                status.state = "turning"
                turn_speed = self.get_parameter("turn_speed").value
                command.angular.z = -turn_speed if msg.angle > 0.0 else turn_speed
            else:
                status.state = "forward"
                command.linear.x = self.get_parameter("forward_speed").value
            self.cmd_vel.publish(command)

        self.status.publish(status)

    def on_enable(self, request, response):
        if request.data:
            self.enabled = True
            response.message = "turn_away enabled"
        else:
            self.stop()
            response.message = "turn_away stopped"

        response.success = True
        self.get_logger().info(response.message)
        return response

    def stop(self):
        if self.enabled:
            self.cmd_vel.publish(Twist())
        self.enabled = False

    def on_set_turn_speed(self, request, response):
        max_turn_speed = self.get_parameter("max_turn_speed").value
        if not 0.0 < request.turn_speed <= max_turn_speed:
            response.success = False
            response.message = (
                f"turn speed must be above 0 and at most {max_turn_speed:.2f} rad/s"
            )
        else:
            self.set_parameters([
                Parameter("turn_speed", value=float(request.turn_speed)),
            ])
            response.success = True
            response.message = f"turn speed set to {request.turn_speed:.2f} rad/s"

        self.get_logger().info(response.message)
        return response


def main():
    rclpy.init()
    node = TurnAway()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    if rclpy.ok():
        node.stop()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
