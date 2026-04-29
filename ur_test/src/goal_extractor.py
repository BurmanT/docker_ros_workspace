#!/usr/bin/env python3
"""
goal_extractor.py

Subscribes to TurtleBot /odom and publishes the pole tip
position as /kinova_goal_pose for the Kinova arm to move to.

Run this on your Kinova laptop inside Docker.

Usage:
  ros2 run ur_test goal_extractor.py
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
import math


# Height of pole tip above TurtleBot base in meters
# Measure this physically
POLE_HEIGHT = 0.15

# Offset from map/odom origin to Kinova base_link (meters)
# Step 1: Place TurtleBot at Kinova base position
# Step 2: Run: ros2 topic echo /odom --once
# Step 3: Note x, y and put them here
MAP_TO_BASE_X = 2.2585
MAP_TO_BASE_Y = 0.0488


class GoalExtractor(Node):
    def __init__(self):
        super().__init__("goal_extractor")

        self._goal_pub = self.create_publisher(
            PoseStamped, "/kinova_goal_pose", 10
        )

        # Service to manually trigger goal update
        self.create_service(
            Trigger, "update_kinova_goal", self._trigger_callback
        )

        self.create_subscription(
            Odometry, "/odom", self._odom_callback, 10
        )

        self._latest_odom = None
        self.get_logger().info("Goal extractor ready.")
        self.get_logger().info(f"Pole height: {POLE_HEIGHT}m")
        self.get_logger().info(
            "Drive TurtleBot near Kinova then call "
            "/update_kinova_goal to send goal to arm."
        )

    # ------------------------------------------------------------------
    def _odom_callback(self, msg: Odometry):
        self._latest_odom = msg

    # ------------------------------------------------------------------
    def _trigger_callback(self, request, response):
        if self._latest_odom is None:
            self.get_logger().error("No odometry received yet!")
            response.success = False
            response.message = "No odometry received"
            return response

        msg = self._latest_odom

        # TurtleBot position in odom frame
        tb_x = msg.pose.pose.position.x
        tb_y = msg.pose.pose.position.y

        # Transform to Kinova base_link frame
        goal_x = tb_x - MAP_TO_BASE_X
        goal_y = tb_y - MAP_TO_BASE_Y
        goal_z = POLE_HEIGHT

        goal = PoseStamped()
        goal.header.frame_id    = "base_link"
        goal.header.stamp       = self.get_clock().now().to_msg()
        goal.pose.position.x    = goal_x
        goal.pose.position.y    = goal_y
        goal.pose.position.z    = goal_z
        goal.pose.orientation.w = 0.0
        goal.pose.orientation.x = 1.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0

        self._goal_pub.publish(goal)

        self.get_logger().info(
            f"TurtleBot at ({tb_x:.2f}, {tb_y:.2f}) → "
            f"Kinova goal: ({goal_x:.2f}, {goal_y:.2f}, {goal_z:.2f})"
        )

        response.success = True
        response.message = f"Goal sent: ({goal_x:.2f}, {goal_y:.2f}, {goal_z:.2f})"
        return response


# ------------------------------------------------------------------
def main():
    rclpy.init()
    node = GoalExtractor()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()