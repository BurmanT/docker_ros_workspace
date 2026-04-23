#!/usr/bin/env python3
"""
demonstration_recorder.py

Listens to /joy and toggles rosbag recording of /joint_states
when the RB button (index 5) is pressed.

Press RB → starts recording  → demo_YYYY-MM-DD_HH-MM-SS/
Press RB → stops recording
"""
import os
import subprocess
import signal
from datetime import datetime

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


RB_BUTTON_INDEX = 5          # RB on Xbox controller
RECORD_TOPICS   = ["/joint_states", "/joint_trajectory_controller/joint_trajectory"]
SAVE_DIR        = os.path.expanduser("~/demonstrations")


class DemonstrationRecorder(Node):
    def __init__(self):
        super().__init__("demonstration_recorder")

        self._recording   = False
        self._bag_process = None
        self._last_rb     = 0   # debounce: previous button state

        os.makedirs(SAVE_DIR, exist_ok=True)

        self._joy_sub = self.create_subscription(
            Joy, "/joy", self._joy_callback, 10
        )

        self.get_logger().info("Demonstration recorder ready.")
        self.get_logger().info(f"Saving bags to: {SAVE_DIR}")
        self.get_logger().info("Press RB to start/stop recording.")

    # ------------------------------------------------------------------
    def _joy_callback(self, msg: Joy):
        rb = msg.buttons[RB_BUTTON_INDEX]

        # Only trigger on the rising edge (button just pressed)
        if rb == 1 and self._last_rb == 0:
            if self._recording:
                self._stop_recording()
            else:
                self._start_recording()

        self._last_rb = rb

    # ------------------------------------------------------------------
    def _start_recording(self):
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        bag_path  = os.path.join(SAVE_DIR, f"demo_{timestamp}")

        cmd = ["ros2", "bag", "record", "-o", bag_path] + RECORD_TOPICS

        self._bag_process = subprocess.Popen(cmd)
        self._recording   = True

        self.get_logger().info(f"Recording started → {bag_path}")
        self.get_logger().info(f"Topics: {RECORD_TOPICS}")
        self.get_logger().info("Press RB again to stop.")

    # ------------------------------------------------------------------
    def _stop_recording(self):
        if self._bag_process is not None:
            self._bag_process.send_signal(signal.SIGINT)
            self._bag_process.wait()
            self._bag_process = None

        self._recording = False
        self.get_logger().info("Recording stopped. Demonstration saved.")
        self.get_logger().info("Press RB to start a new recording.")

    # ------------------------------------------------------------------
    def destroy_node(self):
        if self._recording:
            self.get_logger().info("Shutting down — stopping recording.")
            self._stop_recording()
        super().destroy_node()


# ----------------------------------------------------------------------
def main():
    rclpy.init()
    node = DemonstrationRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()