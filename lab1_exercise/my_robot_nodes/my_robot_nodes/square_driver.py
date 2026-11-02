#!/usr/bin/env python3
"""Drive four sides and four left turns using two action servers."""

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from lab_interfaces.action import DriveDistance
from my_robot_interfaces.action import Rotate
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node


class SquareDriver(Node):
    def __init__(self):
        super().__init__("square_driver")
        self.declare_parameter("side_length", 0.5)
        self.drive = ActionClient(self, DriveDistance, "drive_distance")
        self.rotate = ActionClient(self, Rotate, "rotate")
        self.cmd_vel = self.create_publisher(Twist, "cmd_vel", 10)
        self.robot_ready = False
        self.create_subscription(Odometry, "odom", self.on_odom, 10)
        self.side = 0
        self.waiting = False
        self.timer = self.create_timer(0.2, self.start)

    def start(self):
        if self.waiting:
            return
        if not self.robot_ready:
            return
        if self.side >= 4:
            self.timer.cancel()
            self.get_logger().info("Square completed")
            return
        self.waiting = True
        if not self.drive.wait_for_server(timeout_sec=1.0):
            self.fail(f"drive action unavailable on side {self.side + 1}")
            return
        goal = DriveDistance.Goal()
        goal.distance = float(self.get_parameter("side_length").value)
        future = self.drive.send_goal_async(goal)
        future.add_done_callback(self.on_drive_goal)

    def on_odom(self, _msg):
        self.robot_ready = True

    def on_drive_goal(self, future):
        handle = future.result()
        if not handle.accepted:
            self.fail(f"drive goal rejected on side {self.side + 1}")
            return
        result_future = handle.get_result_async()
        result_future.add_done_callback(self.on_drive_result)

    def on_drive_result(self, future):
        result = future.result()
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            self.fail(f"drive failed on side {self.side + 1}")
            return
        if not self.rotate.wait_for_server(timeout_sec=1.0):
            self.fail(f"rotate action unavailable on side {self.side + 1}")
            return
        goal = Rotate.Goal()
        goal.angle = 1.5708
        future = self.rotate.send_goal_async(goal)
        future.add_done_callback(self.on_rotate_goal)

    def on_rotate_goal(self, future):
        handle = future.result()
        if not handle.accepted:
            self.fail(f"rotate goal rejected on side {self.side + 1}")
            return
        result_future = handle.get_result_async()
        result_future.add_done_callback(self.on_rotate_result)

    def on_rotate_result(self, future):
        if future.result().status != GoalStatus.STATUS_SUCCEEDED:
            self.fail(f"rotate failed on side {self.side + 1}")
            return
        self.side += 1
        self.waiting = False

    def fail(self, message):
        self.cmd_vel.publish(Twist())
        self.get_logger().error(message)
        self.waiting = True


def main():
    rclpy.init()
    node = SquareDriver()
    try:
        rclpy.spin(node, executor=MultiThreadedExecutor())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
