#!/usr/bin/env python3
"""
joy_teleop.py

Teleoperate the Kinova Gen3 Lite with an Xbox controller.
Publishes position commands at a fixed rate to hold joints
against gravity even when not moving.

Control scheme:
  LB (hold)           — deadman switch
  RB                  — start/stop rosbag recording
  A button            — close gripper
  B button            — open gripper

  Left  stick RIGHT   — joint 1 clockwise (base rotates right)
  Left  stick LEFT    — joint 1 counter-clockwise
  Left  stick UP      — joint 2 shoulder up
  Left  stick DOWN    — joint 2 shoulder down

  Right stick UP/DOWN    — joint 3 arm up/down
  Right stick LEFT/RIGHT — joint 4 wrist 1

  LT (pull)           — joint 5 positive
  RT (pull)           — joint 5 negative
"""
import os
import signal
import subprocess
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Joy, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import GripperCommand
from builtin_interfaces.msg import Duration


# ------------------------------------------------------------------
# Tuning
# ------------------------------------------------------------------
JOINT_NAMES = [
    "joint_1", "joint_2", "joint_3",
    "joint_4", "joint_5", "joint_6",
]

JOINT_STEP    = 0.0025  # radians per cycle at full stick deflection
DEADZONE      = 0.08   # ignore stick values below this
PUBLISH_HZ    = 50.0   # how often to publish hold/move commands
TRAJECTORY_DT = 0.05   # seconds for each trajectory point

JOINT_LIMITS = [
    (-2.61, 2.61),
    (-2.61, 2.61),
    (-2.61, 2.61),
    (-2.61, 2.61),
    (-2.53, 2.53),
    (-2.61, 2.61),
]

# Buttons
BTN_A  = 0
BTN_B  = 1
BTN_LB = 4
BTN_RB = 5

# Axes
AXIS_LX = 0
AXIS_LY = 1
AXIS_LT = 2
AXIS_RX = 3
AXIS_RY = 4
AXIS_RT = 5

# Gripper positions
GRIPPER_OPEN   = 0.8   # fully open
GRIPPER_CLOSED = 0.0   # fully closed

RECORD_TOPICS = ["/joint_states"]
SAVE_DIR      = os.path.expanduser("~/demonstrations")


def apply_deadzone(value: float, deadzone: float) -> float:
    if abs(value) < deadzone:
        return 0.0
    sign = 1.0 if value > 0 else -1.0
    return sign * (abs(value) - deadzone) / (1.0 - deadzone)


class JoyTeleop(Node):
    def __init__(self):
        super().__init__("joy_teleop")

        self._joint_positions          = [0.0] * 6
        self._target_positions         = [0.0] * 6
        self._joint_positions_received = False

        self._last_rb_btn  = 0
        self._last_a_btn   = 0
        self._last_b_btn   = 0
        self._recording    = False
        self._bag_process  = None

        # Current joy state
        self._lx = 0.0
        self._ly = 0.0
        self._rx = 0.0
        self._ry = 0.0
        self._lt = 0.0
        self._rt = 0.0
        self._lb = False

        os.makedirs(SAVE_DIR, exist_ok=True)

        # Joint trajectory publisher
        self._joint_pub = self.create_publisher(
            JointTrajectory,
            "/joint_trajectory_controller/joint_trajectory",
            10,
        )

        # Gripper action client
        self._gripper_client = ActionClient(
            self,
            GripperCommand,
            "/gen3_lite_2f_gripper_controller/gripper_cmd",
        )

        self.create_subscription(Joy, "/joy", self._joy_callback, 10)
        self.create_subscription(
            JointState, "/joint_states", self._joint_state_callback, 10
        )

        # Timer — publishes at fixed rate to hold joints against gravity
        self.create_timer(1.0 / PUBLISH_HZ, self._control_loop)

        self.get_logger().info("Joy teleop ready — waiting for /joint_states...")
        self.get_logger().info("Hold LB to move. Press RB to record.")
        self.get_logger().info("Press A to close gripper, B to open gripper.")

    # ------------------------------------------------------------------
    def _joint_state_callback(self, msg: JointState):
        if not self._joint_positions_received:
            self.get_logger().info("Joint states received — ready!")
            for i, name in enumerate(JOINT_NAMES):
                if name in msg.name:
                    idx = msg.name.index(name)
                    self._target_positions[i] = msg.position[idx]
            self._joint_positions_received = True

        for i, name in enumerate(JOINT_NAMES):
            if name in msg.name:
                idx = msg.name.index(name)
                self._joint_positions[i] = msg.position[idx]

    # ------------------------------------------------------------------
    def _joy_callback(self, msg: Joy):
        buttons = msg.buttons
        axes    = msg.axes

        self._lb = bool(buttons[BTN_LB])

        # RB — toggle recording
        rb = buttons[BTN_RB]
        if rb == 1 and self._last_rb_btn == 0:
            self._stop_recording() if self._recording else self._start_recording()
        self._last_rb_btn = rb

        # A — close gripper
        a_btn = buttons[BTN_A]
        if a_btn == 1 and self._last_a_btn == 0:
            self._send_gripper(GRIPPER_CLOSED)
        self._last_a_btn = a_btn

        # B — open gripper
        b_btn = buttons[BTN_B]
        if b_btn == 1 and self._last_b_btn == 0:
            self._send_gripper(GRIPPER_OPEN)
        self._last_b_btn = b_btn

        # Store axis values
        self._lx = apply_deadzone( axes[AXIS_LX], DEADZONE)
        self._ly = apply_deadzone( axes[AXIS_LY], DEADZONE)
        self._rx = apply_deadzone( axes[AXIS_RX], DEADZONE)
        self._ry = apply_deadzone( axes[AXIS_RY], DEADZONE)
        self._lt = apply_deadzone((1.0 - axes[AXIS_LT]) / 2.0, DEADZONE)
        self._rt = apply_deadzone((1.0 - axes[AXIS_RT]) / 2.0, DEADZONE)

    # ------------------------------------------------------------------
    def _control_loop(self):
        """Runs at PUBLISH_HZ — updates target positions and publishes."""
        if not self._joint_positions_received:
            return

        if self._lb:
            deltas = [
                 self._lx * JOINT_STEP,                    # joint 1: base
                 self._ly * JOINT_STEP,                    # joint 2: shoulder
                -self._ry * JOINT_STEP,                    # joint 3: arm up/down
                 self._rx * JOINT_STEP,                    # joint 4: wrist 1
                (self._lt - self._rt) * JOINT_STEP,        # joint 5: LT=+, RT=-
                 0.0,                                       # joint 6: unused
            ]

            for i, delta in enumerate(deltas):
                lo, hi = JOINT_LIMITS[i]
                self._target_positions[i] = max(
                    lo, min(hi, self._target_positions[i] + delta)
                )

        # Always publish to hold against gravity
        point = JointTrajectoryPoint()
        point.positions       = list(self._target_positions)
        point.time_from_start = Duration(
            sec=0, nanosec=int(TRAJECTORY_DT * 1e9)
        )

        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names  = JOINT_NAMES
        traj.points       = [point]

        self._joint_pub.publish(traj)

    # ------------------------------------------------------------------
    def _send_gripper(self, position: float):
        if not self._gripper_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("Gripper action server not available")
            return
        goal = GripperCommand.Goal()
        goal.command.position   = position
        goal.command.max_effort = 100.0
        self._gripper_client.send_goal_async(goal)
        self.get_logger().info(
            f"Gripper {'closing' if position == GRIPPER_CLOSED else 'opening'}"
        )

    # ------------------------------------------------------------------
    def _start_recording(self):
        timestamp         = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        bag_path          = os.path.join(SAVE_DIR, f"demo_{timestamp}")
        cmd               = ["ros2", "bag", "record", "-o", bag_path] + RECORD_TOPICS
        self._bag_process = subprocess.Popen(cmd)
        self._recording   = True
        self.get_logger().info(f"Recording started → {bag_path}")
        self.get_logger().info("Press RB again to stop.")

    def _stop_recording(self):
        if self._bag_process:
            self._bag_process.send_signal(signal.SIGINT)
            self._bag_process.wait()
            self._bag_process = None
        self._recording = False
        self.get_logger().info("Recording stopped.")

    def destroy_node(self):
        if self._recording:
            self._stop_recording()
        super().destroy_node()


# ------------------------------------------------------------------
def main():
    rclpy.init()
    node = JoyTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()