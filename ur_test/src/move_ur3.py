#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from moveit.planning import MoveItPy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32
from std_srvs.srv import Trigger

# ---------------------------------------------------------------------------
# Hardcoded arm poses for each TurtleBot position
# Adjust x, y, z, and orientation once you know the real targets
# ---------------------------------------------------------------------------
## (0.0, 0.0, 0.0, 1.0)
ARM_POSES = {
    0: {
        "position":    (0.4, 0.0, 0.4),
        "orientation": (0.0, 1.0, 0.0, 0.0),
    },
    
    1: {
        "position":    (0.4, 0.2, 0.4),
        "orientation": (0.0, 1.0, 0.0, 0.0),
    },
    
}


class MoveNode(Node):
    def __init__(self):
        super().__init__("moveit_py_node")

        # ReentrantCallbackGroup allows callbacks to run concurrently
        # so planning/execution won't block incoming messages
        self._cb_group = ReentrantCallbackGroup()

        self.get_logger().info("Initialising MoveItPy...")
        self._moveit_py = MoveItPy(node_name="moveit_py_node")
        self._arm       = self._moveit_py.get_planning_component("arm")
        self.get_logger().info("MoveItPy ready.")

        self._goal_pose = None
        self._is_moving = False  # simple guard to avoid concurrent moves

        # ------------------------------------------------------------------
        # Subscribe to /turtlebot_goal_index (latched — matches TurtleBot QoS)
        # ------------------------------------------------------------------
        latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            Int32,
            "/turtlebot_goal_index",
            self._turtlebot_index_callback,
            latched_qos,
            callback_group=self._cb_group,
        )

        # Subscribe to goal from goal_extractor.py (kept for compatibility)
        self.create_subscription(
            PoseStamped,
            "/kinova_goal_pose",
            self._goal_callback,
            10,
            callback_group=self._cb_group,
        )

        # Manual trigger service
        self.create_service(
            Trigger,
            "move_to_pose",
            self.move_to_pose_callback,
            callback_group=self._cb_group,
        )

        self.get_logger().info("MoveNode ready.")
        self.get_logger().info("Subscribed to /turtlebot_goal_index (0 or 1).")

    # ------------------------------------------------------------------
    def _turtlebot_index_callback(self, msg: Int32):
        index = msg.data
        self.get_logger().info(f"Received TurtleBot goal index: {index}")

        if index not in ARM_POSES:
            self.get_logger().error(f"Unknown index {index} — no pose defined.")
            return

        if self._is_moving:
            self.get_logger().warn("Arm is already moving — ignoring new goal.")
            return

        # Build the target PoseStamped from the hardcoded dict
        pose = ARM_POSES[index]
        px, py, pz     = pose["position"]
        ox, oy, oz, ow = pose["orientation"]

        target = PoseStamped()
        target.header.stamp    = self.get_clock().now().to_msg()
        target.header.frame_id = "base_link"
        target.pose.position.x    = px
        target.pose.position.y    = py
        target.pose.position.z    = pz
        target.pose.orientation.x = ox
        target.pose.orientation.y = oy
        target.pose.orientation.z = oz
        target.pose.orientation.w = ow

        self._goal_pose = target
        self.get_logger().info(
            f"Moving arm to pose for index {index}: "
            f"({px:.2f}, {py:.2f}, {pz:.2f})"
        )
        self.go_to_pose()

    # ------------------------------------------------------------------
    def _goal_callback(self, msg: PoseStamped):
        if self._is_moving:
            self.get_logger().warn("Arm is already moving — ignoring new goal.")
            return

        self._goal_pose = msg
        self.get_logger().info(
            f"New goal received from /kinova_goal_pose: "
            f"({msg.pose.position.x:.2f}, "
            f"{msg.pose.position.y:.2f}, "
            f"{msg.pose.position.z:.2f})"
        )
        self.go_to_pose()

    # ------------------------------------------------------------------
    def move_to_pose_callback(self, request, response):
        success = self.go_to_pose()
        response.success = success
        response.message = "Succeeded" if success else "Failed"
        return response

    # ------------------------------------------------------------------
    def go_to_pose(self) -> bool:
        if self._goal_pose is None:
            self.get_logger().error("No goal pose set yet.")
            return False

        if self._is_moving:
            self.get_logger().warn("go_to_pose called while already moving — skipping.")
            return False

        self._is_moving = True
        try:
            self.get_logger().info(
                f"Planning to position: "
                f"x={self._goal_pose.pose.position.x:.3f}, "
                f"y={self._goal_pose.pose.position.y:.3f}, "
                f"z={self._goal_pose.pose.position.z:.3f} "
                f"in frame '{self._goal_pose.header.frame_id}'"
            )

            self._arm.set_start_state_to_current_state()
            self.get_logger().info("Start state set to current state.")

            self._arm.set_goal_state(
                pose_stamped_msg=self._goal_pose,
                pose_link="end_effector_link",
            )
            self.get_logger().info("Goal state set — calling plan()...")

            plan_result = self._arm.plan()

            if not plan_result:
                self.get_logger().error(
                    "Planning failed. Check: \n"
                    "  1. Is end_effector_link the correct link name?\n"
                    "  2. Is the target pose reachable?\n"
                    "  3. Is joint_state_broadcaster active?\n"
                    f"  Plan result was: {plan_result}"
                )
                return False

            self.get_logger().info(
                f"Planning succeeded (type={type(plan_result)}). Executing trajectory..."
            )

            self._moveit_py.execute(
                plan_result.trajectory,
                controllers=["joint_trajectory_controller"],
            )

            self.get_logger().info("Execution complete.")
            return True

        except Exception as e:
            self.get_logger().error(f"Exception in go_to_pose: {type(e).__name__}: {e}")
            return False

        finally:
            self._is_moving = False


# ------------------------------------------------------------------
def main():
    rclpy.init()
    node = MoveNode()
    # Increased to 4 threads so planning/execution don't starve callbacks
    executor = rclpy.executors.MultiThreadedExecutor(4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

"""
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from moveit.planning import MoveItPy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32
from std_srvs.srv import Trigger
#  (0.4, 0.0, 0.4), 

# ---------------------------------------------------------------------------
# Hardcoded arm poses for each TurtleBot position
# Adjust x, y, z, and orientation once you know the real targets
# ---------------------------------------------------------------------------
ARM_POSES = {
    0: {
        "position":    (0.4, 0.0, 0.4),   # position when TurtleBot is at goal 0
        "orientation": (0.0, 0.0, 0.0, 1.0),  # 3(x, y, z, w)
    },
    1: {
        "position":    (0.4, 0.2, 0.4),   # position when TurtleBot is at goal 1
        "orientation": (0.0, 0.0, 0.0, 1.0),
    },
}


class MoveNode(Node):
    def __init__(self):
        super().__init__("moveit_py_node")

        self._moveit_py = MoveItPy(node_name="moveit_py_node")
        self._arm       = self._moveit_py.get_planning_component("arm")

        self._goal_pose = None

        # ------------------------------------------------------------------
        # Subscribe to /turtlebot_goal_index (latched — matches TurtleBot QoS)
        # ------------------------------------------------------------------
        latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            Int32,
            "/turtlebot_goal_index",
            self._turtlebot_index_callback,
            latched_qos,
        )

        # Subscribe to goal from goal_extractor.py (kept for compatibility)
        self.create_subscription(
            PoseStamped,
            "/kinova_goal_pose",
            self._goal_callback,
            10,
        )

        # Manual trigger service
        self.create_service(
            Trigger, "move_to_pose", self.move_to_pose_callback
        )

        self.get_logger().info("MoveNode ready.")
        self.get_logger().info("Subscribed to /turtlebot_goal_index (0 or 1).")

    # ------------------------------------------------------------------
    def _turtlebot_index_callback(self, msg: Int32):
        index = msg.data
        self.get_logger().info(f"Received TurtleBot goal index: {index}")

        if index not in ARM_POSES:
            self.get_logger().error(f"Unknown index {index} — no pose defined.")
            return

        # Build the target PoseStamped from the hardcoded dict
        pose = ARM_POSES[index]
        px, py, pz       = pose["position"]
        ox, oy, oz, ow   = pose["orientation"]

        target = PoseStamped()
        target.header.frame_id      = "base_link"
        target.pose.position.x      = px
        target.pose.position.y      = py
        target.pose.position.z      = pz
        target.pose.orientation.x   = ox
        target.pose.orientation.y   = oy
        target.pose.orientation.z   = oz
        target.pose.orientation.w   = ow

        self._goal_pose = target
        self.get_logger().info(
            f"Moving arm to pose for index {index}: "
            f"({px:.2f}, {py:.2f}, {pz:.2f})"
        )
        self.go_to_pose()

    # ------------------------------------------------------------------
    def _goal_callback(self, msg: PoseStamped):
        self._goal_pose = msg
        self.get_logger().info(
            f"New goal received from /kinova_goal_pose: "
            f"({msg.pose.position.x:.2f}, "
            f"{msg.pose.position.y:.2f}, "
            f"{msg.pose.position.z:.2f})"
        )
        self.go_to_pose()

    # ------------------------------------------------------------------
    def move_to_pose_callback(self, request, response):
        success = self.go_to_pose()
        response.success = success
        response.message = "Succeeded" if success else "Failed"
        return response

    # ------------------------------------------------------------------
    def go_to_pose(self) -> bool:
        if self._goal_pose is None:
            self.get_logger().error("No goal pose set yet.")
            return False

        self._arm.set_start_state_to_current_state()
        self._arm.set_goal_state(
            pose_stamped_msg=self._goal_pose,
            pose_link="end_effector_link",
        )

        plan_result = self._arm.plan()
        if plan_result:
            self.get_logger().info("Planning succeeded — executing.")
            self._moveit_py.execute(plan_result.trajectory, controllers=["joint_trajectory_controller"])
            return True

        self.get_logger().error("Planning failed.")
        return False


# ------------------------------------------------------------------
def main():
    rclpy.init()
    node = MoveNode()
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
"""