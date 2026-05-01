#!/usr/bin/env python3
"""
goal_extractor.py

Runs on TurtleBot laptop (ROS2 Humble).
Publishes goal as Float64MultiArray which crosses ROS2 versions reliably.

Message format: [x, y, z, qx, qy, qz, qw]

Usage:
  python3 ~/tanu_workspace/goal_extractor.py

Trigger:
  ros2 service call /update_kinova_goal std_srvs/srv/Trigger {}
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger


# ------------------------------------------------------------------
# Configuration
# ------------------------------------------------------------------

# Kinova base position in map frame
# Measured by driving TurtleBot to Kinova base and reading /amcl_pose
KINOVA_BASE_MAP_X = 0.1783
KINOVA_BASE_MAP_Y = -0.3429

# Height above Kinova base level (meters)
POLE_HEIGHT = 0.15

# Gripper orientation — pointing down
ORIENTATION_X = 1.0
ORIENTATION_Y = 0.0
ORIENTATION_Z = 0.0
ORIENTATION_W = 0.0


class GoalExtractor(Node):
    def __init__(self):
        super().__init__("goal_extractor")

        # Publisher — Float64MultiArray crosses ROS2 versions reliably
        # Format: [x, y, z, qx, qy, qz, qw]
        self._goal_pub = self.create_publisher(
            Float64MultiArray, "/kinova_goal_pose", 10
        )

        # Service — trigger when TurtleBot arrives at goal
        self.create_service(
            Trigger, "update_kinova_goal", self._trigger_callback
        )

        # Subscriber — reads TurtleBot position from AMCL
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self._amcl_callback,
            10,
        )

        self._latest_pose = None

        self.get_logger().info("Goal extractor ready.")
        self.get_logger().info(
            f"Kinova base in map frame: "
            f"({KINOVA_BASE_MAP_X}, {KINOVA_BASE_MAP_Y})"
        )
        self.get_logger().info(f"Pole height: {POLE_HEIGHT}m")
        self.get_logger().info(
            "Waiting for /amcl_pose... "
            "Call /update_kinova_goal when TurtleBot is at goal."
        )

    # ------------------------------------------------------------------
    def _amcl_callback(self, msg: PoseWithCovarianceStamped):
        self._latest_pose = msg

    # ------------------------------------------------------------------
    def _trigger_callback(self, request, response):
        if self._latest_pose is None:
            self.get_logger().error("No AMCL pose received yet!")
            self.get_logger().error(
                "Make sure Nav2 is running and "
                "2D Pose Estimate has been set in RViz."
            )
            response.success = False
            response.message = "No AMCL pose received"
            return response

        # TurtleBot position in map frame
        tb_x = self._latest_pose.pose.pose.position.x
        tb_y = self._latest_pose.pose.pose.position.y

        # Convert to Kinova base_link frame
        goal_x = tb_x - KINOVA_BASE_MAP_X
        goal_y = tb_y - KINOVA_BASE_MAP_Y
        goal_z = POLE_HEIGHT

        # Publish as Float64MultiArray [x, y, z, qx, qy, qz, qw]
        msg = Float64MultiArray()
        msg.data = [
            goal_x, goal_y, goal_z,
            ORIENTATION_X, ORIENTATION_Y,
            ORIENTATION_Z, ORIENTATION_W
        ]
        self._goal_pub.publish(msg)

        self.get_logger().info(
            f"TurtleBot in map frame:   ({tb_x:.3f}, {tb_y:.3f})"
        )
        self.get_logger().info(
            f"Kinova goal in base_link: "
            f"({goal_x:.3f}, {goal_y:.3f}, {goal_z:.3f})"
        )

        response.success = True
        response.message = (
            f"Goal sent: ({goal_x:.2f}, {goal_y:.2f}, {goal_z:.2f})"
        )
        return response


# ------------------------------------------------------------------
def main():
    rclpy.init()
    node = GoalExtractor()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()