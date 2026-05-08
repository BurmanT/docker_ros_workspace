"""
ArmProximity.py

Custom ROS2 node to monitor the proximity of the robot's arm to the Turtlebot after it has moved to the goal.

Uses the LaserScan topic to check if the arm is in the Turtlebot's path and publishes a boolean message to the "arm_in_proximity" topic to indicate whether the arm is detected within a certain distance

"""


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class ArmProximity(Node):
    def __init__(self):
        super().__init__("arm_proximity")

        self.scan_subscription = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            10
        )

        self.arm_in_proximity = self.create_publisher(Bool, "arm_in_proximity", 10)
        self.angle_min = -30
        self.angle_max = 30
    
    def scan_callback(self, msg):
        # Check for obstacles in the path of the turtlebot
        obstacle_threshold = 0.5  # meters
        arm_detected = False

        for ind, distance in enumerate(msg.ranges):
            # measure the angle of the obstacle 
            angle = msg.angle_min + ind * msg.angle_increment
            if self.angle_min < angle < self.angle_max and distance < obstacle_threshold:
                self.get_logger().warn("Obstacle/ arm in front of turtlebot!")
                arm_detected = True
                break

        if not arm_detected:
            self.get_logger().info("No arm detected within turtlebot proximity.")
        proximity_msg = Bool()
        proximity_msg.data = arm_detected
        self.arm_in_proximity.publish(proximity_msg)

def main():
    rclpy.init()
    arm_proximity_node = ArmProximity()
    rclpy.spin(arm_proximity_node)
    arm_proximity_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
        