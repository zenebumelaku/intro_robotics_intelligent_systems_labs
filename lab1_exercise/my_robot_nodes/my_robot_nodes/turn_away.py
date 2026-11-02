#!/usr/bin/env python3
"""Drive forward and turn away from the nearest obstacle."""

import math

import rclpy
from geometry_msgs.msg import Twist
from lab_interfaces.msg import ObstacleInfo
from my_robot_interfaces.msg import RobotStatus
from my_robot_interfaces.srv import SetTurnSpeed
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_srvs.srv import SetBool


class TurnAway(Node):
    def __init__(self):
        super().__init__("turn_away")
        self.declare_parameter("forward_speed", 0.2)
        self.declare_parameter("turn_speed", 0.5)
        self.declare_parameter("max_turn_speed", 1.5)
        self.enabled = False
        self.last_distance = 0.0
        self.cmd_vel = self.create_publisher(Twist, "cmd_vel", 10)
        self.status = self.create_publisher(RobotStatus, "robot_status", 10)
        self.create_subscription(ObstacleInfo, "obstacle", self.on_obstacle, 10)
        self.create_service(SetBool, "turn_away_enable", self.on_enable)
        self.create_service(SetTurnSpeed, "set_turn_speed", self.on_set_turn_speed)

    def publish_stop(self):
        self.cmd_vel.publish(Twist())

    def publish_status(self, state):
        msg = RobotStatus()
        msg.state = state
        msg.obstacle_distance = self.last_distance
        self.status.publish(msg)

    def on_obstacle(self, msg):
        self.last_distance = msg.distance
        if not self.enabled:
            self.publish_status("stopped")
            return
        twist = Twist()
        if msg.too_close:
            turn_speed = self.get_parameter("turn_speed").value
            twist.angular.z = -turn_speed if msg.angle > 0 else turn_speed
            self.publish_status("turning")
        else:
            twist.linear.x = self.get_parameter("forward_speed").value
            self.publish_status("forward")
        self.cmd_vel.publish(twist)

    def on_enable(self, request, response):
        self.enabled = request.data
        if not self.enabled:
            self.publish_stop()
            self.publish_status("stopped")
        response.success = True
        response.message = "enabled" if self.enabled else "stopped"
        return response

    def on_set_turn_speed(self, request, response):
        maximum = self.get_parameter("max_turn_speed").value
        invalid_speed = (
            not math.isfinite(request.turn_speed)
            or request.turn_speed <= 0.0
            or request.turn_speed > maximum
        )
        if invalid_speed:
            response.success = False
            response.message = f"turn speed must be above 0 and at most {maximum} rad/s"
        else:
            self.set_parameters([Parameter("turn_speed", value=float(request.turn_speed))])
            response.success = True
            response.message = f"turn speed set to {request.turn_speed:.2f} rad/s"
        return response


def main():
    rclpy.init()
    node = TurnAway()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
