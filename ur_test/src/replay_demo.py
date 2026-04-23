#!/usr/bin/env python3
"""
replay_demo.py

Reads a recorded rosbag (.mcap) of /joint_states and replays it
as JointTrajectory commands to the Kinova arm.

- Smoothly moves to the start position before replaying
- Downsamples high frequency recordings
- Smoothstep interpolation between points for fluid motion

Usage:
  ros2 run ur_test replay_demo.py --ros-args \
    -p bag_path:=/root/demonstrations/demo_<timestamp> \
    -p speed:=1.0
"""
import time

import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions


JOINT_NAMES = [
    "joint_1", "joint_2", "joint_3",
    "joint_4", "joint_5", "joint_6",
]

PUBLISH_HZ         = 50.0   # playback rate
TRAJECTORY_DT      = 1.0 / PUBLISH_HZ
MOVE_TO_START_SECS = 5.0    # seconds to smoothly reach start position
DOWNSAMPLE_FACTOR  = 5     # keep every Nth message


class DemoReplay(Node):
    def __init__(self):
        super().__init__("demo_replay")

        self.declare_parameter("bag_path", "")
        self.declare_parameter("speed", 1.0)

        self._bag_path           = self.get_parameter("bag_path").value
        self._speed              = self.get_parameter("speed").value
        self._current_positions  = None
        self._positions_received = False

        if not self._bag_path:
            self.get_logger().error("No bag_path provided!")
            return

        self._joint_pub = self.create_publisher(
            JointTrajectory,
            "/joint_trajectory_controller/joint_trajectory",
            10,
        )

        self.create_subscription(
            JointState, "/joint_states", self._joint_state_callback, 10
        )

        self.create_timer(2.0, self._run_replay)
        self.get_logger().info(f"Replaying: {self._bag_path}")
        self.get_logger().info(f"Speed: {self._speed}x")

    # ------------------------------------------------------------------
    def _joint_state_callback(self, msg: JointState):
        self._positions_received = True
        positions = []
        for name in JOINT_NAMES:
            if name in msg.name:
                idx = msg.name.index(name)
                positions.append(msg.position[idx])
            else:
                positions.append(0.0)
        self._current_positions = positions

    # ------------------------------------------------------------------
    def _run_replay(self):
        self.destroy_timer(list(self._timers)[0])

        # Wait for current joint positions
        self.get_logger().info("Waiting for joint states...")
        start = time.time()
        while not self._positions_received:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start > 10.0:
                self.get_logger().error("Timed out waiting for joint states")
                return

        # Load and downsample bag
        joint_states = self._read_bag()
        if not joint_states:
            self.get_logger().error("No /joint_states messages found in bag")
            return

        joint_states = joint_states[::DOWNSAMPLE_FACTOR]
        self.get_logger().info(
            f"Loaded {len(joint_states)} messages after downsampling"
        )

        # Get start position from bag
        _, first_msg    = joint_states[0]
        start_positions = self._extract_positions(first_msg)

        # Smoothly move to start position
        self.get_logger().info(
            f"Moving to start position over {MOVE_TO_START_SECS}s..."
        )
        self._smooth_move_to(
            self._current_positions, start_positions, MOVE_TO_START_SECS
        )

        self.get_logger().info("At start. Replaying in 2 seconds...")
        time.sleep(2.0)
        self.get_logger().info("Replaying!")

        # Replay with smoothstep interpolation between points
        prev_time      = None
        prev_positions = start_positions

        for timestamp, msg in joint_states:
            target = self._extract_positions(msg)

            if prev_time is not None:
                dt    = (timestamp - prev_time) / 1e9 / self._speed
                steps = max(1, int(dt * PUBLISH_HZ))
                for i in range(steps):
                    alpha  = (i + 1) / steps
                    t      = alpha * alpha * (3.0 - 2.0 * alpha)  # smoothstep
                    interp = [
                        prev_positions[j] + t * (target[j] - prev_positions[j])
                        for j in range(6)
                    ]
                    self._send_positions(interp)
                    time.sleep(TRAJECTORY_DT/self._speed)
            else:
                self._send_positions(target)

            prev_time      = timestamp
            prev_positions = target

        self.get_logger().info("Replay complete!")

    # ------------------------------------------------------------------
    def _smooth_move_to(self, start: list, end: list, duration: float):
        """Smoothstep interpolation from start to end over duration seconds."""
        steps    = int(duration * PUBLISH_HZ)
        interval = duration / steps
        for i in range(steps + 1):
            t      = i / steps
            alpha  = t * t * (3.0 - 2.0 * t)
            interp = [start[j] + alpha * (end[j] - start[j]) for j in range(6)]
            self._send_positions(interp)
            time.sleep(interval)

    # ------------------------------------------------------------------
    def _extract_positions(self, msg: JointState) -> list:
        positions = []
        for name in JOINT_NAMES:
            if name in msg.name:
                idx = msg.name.index(name)
                positions.append(msg.position[idx])
            else:
                positions.append(0.0)
        return positions

    # ------------------------------------------------------------------
    def _send_positions(self, positions: list):
        point = JointTrajectoryPoint()
        point.positions       = positions
        point.time_from_start = Duration(
            sec=0, nanosec=int(TRAJECTORY_DT * 1e9)
        )
        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names  = JOINT_NAMES
        traj.points       = [point]
        self._joint_pub.publish(traj)

    # ------------------------------------------------------------------
    def _read_bag(self):
        storage_options   = StorageOptions(uri=self._bag_path, storage_id="mcap")
        converter_options = ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        )
        reader = SequentialReader()
        reader.open(storage_options, converter_options)

        topic_types = reader.get_all_topics_and_types()
        type_map    = {t.name: t.type for t in topic_types}

        if "/joint_states" not in type_map:
            self.get_logger().error("No /joint_states topic in bag")
            return []

        joint_states = []
        while reader.has_next():
            topic, data, timestamp = reader.read_next()
            if topic == "/joint_states":
                msg = deserialize_message(data, JointState)
                joint_states.append((timestamp, msg))

        return joint_states


# ------------------------------------------------------------------
def main():
    rclpy.init()
    node = DemoReplay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()