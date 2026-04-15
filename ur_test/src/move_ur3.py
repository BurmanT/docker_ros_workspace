#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit.planning import MoveItPy
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger


class MoveNode(Node):
    def __init__(self):
        super().__init__("moveit_py_node")

        # Config is now loaded from the parameter server via the launch file.
        # The launch file passes moveit_config.to_dict() + planning_pipelines
        # + plan_request_params — MoveItPy can find everything it needs.
        self._moveit_py = MoveItPy(node_name="moveit_py_node")

        # Verify planning group name against your SRDF:
        #   grep 'group name' <your_moveit_config>/config/*.srdf
        self._arm = self._moveit_py.get_planning_component("arm")

        self.create_service(Trigger, "move_to_pose", self.move_to_pose_callback)
        self.get_logger().info("MoveNode ready — call /move_to_pose to trigger.")

    def move_to_pose_callback(self, request, response):
        success = self.go_to_pose()
        response.success = success
        response.message = "Execution succeeded" if success else "Planning failed"
        return response

    def go_to_pose(self) -> bool:
        target_pose = PoseStamped()
        target_pose.header.frame_id = "base_link"
        target_pose.pose.position.x = 0.5
        target_pose.pose.position.y = 0.0
        target_pose.pose.position.z = 0.5
        target_pose.pose.orientation.w = 1.0

        self._arm.set_start_state_to_current_state()
        self._arm.set_goal_state(
            pose_stamped_msg=target_pose,
            pose_link="end_effector_link",  # verify against your SRDF tip link
        )

        plan_result = self._arm.plan()

        if plan_result:
            self.get_logger().info("Planning succeeded — executing.")
            self._moveit_py.execute(plan_result.trajectory, controllers=[])
            return True

        self.get_logger().error("Planning failed.")
        return False


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