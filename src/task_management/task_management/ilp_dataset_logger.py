#!/usr/bin/env python3
"""
ILP Dataset Logger Node

Subscribes to a structured task-allocation event topic and writes ILP-ready CSV.

This node is intentionally minimal:
- No GUI forwarding
- No generic logs
- Only structured ILP training data

Expected incoming message on /ilp/task_allocation_event:
- String payload in JSON format

JSON schema expected:
{
  "event": "task_allocation",
  "timestamp": "optional human time string",
  "task": {
    "id": "t17",
    "priority": "high",
    "type": "delivery",
    "location": "shelf_5"
  },
  "assigned_robot": "robot_2",
  "candidates": [
    {"robot": "robot_1", "battery": 55, "idle": true, "workload": 1, "distance": 8.1},
    {"robot": "robot_2", "battery": 78, "idle": true, "workload": 0, "distance": 2.3}
  ]
}

This node converts continuous values into buckets:
- battery_bucket: low/medium/high
- distance_bucket: near/medium/far

Then it writes N rows per allocation:
- one row per candidate robot
- label=1 for assigned robot, else label=0
"""

import os
import csv
import json
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def battery_bucket(b: float) -> str:
    # Tune these thresholds later if needed
    if b < 30:
        return "low"
    if b < 60:
        return "medium"
    return "high"


def distance_bucket(d: float) -> str:
    # Tune these thresholds to your map scale
    if d <= 3.0:
        return "near"
    if d <= 7.0:
        return "medium"
    return "far"


class ILPDatasetLogger(Node):
    def __init__(self):
        super().__init__("ilp_dataset_logger")

        # Parameters (override via launch later)
        self.declare_parameter("out_dir", "ilp_logs")
        self.declare_parameter("file_prefix", "ilp_dataset")
        self.declare_parameter("topic", "/ilp/task_allocation_event")

        out_dir = self.get_parameter("out_dir").get_parameter_value().string_value
        file_prefix = self.get_parameter("file_prefix").get_parameter_value().string_value
        topic = self.get_parameter("topic").get_parameter_value().string_value

        # Keep it local to workspace by default; can be changed in launch
        out_dir = os.path.abspath(out_dir)
        os.makedirs(out_dir, exist_ok=True)

        ts = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.csv_path = os.path.join(out_dir, f"{file_prefix}_{ts}.csv")

        self.headers = [
            "timestamp",
            "task_id",
            "task_location",
            "candidate_robot",
            "assigned_robot",
            "idle",
            "workload",
            "battery",
            "battery_bucket",
            "distance",
            "distance_bucket",
            "score",
            "is_chosen",
            "label"
        ]

        with open(self.csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(self.headers)

        self.get_logger().info(f"[ILP Dataset Logger] Writing: {self.csv_path}")
        self.get_logger().info(f"[ILP Dataset Logger] Subscribed to: {topic}")

        self.sub = self.create_subscription(String, topic, self.cb, 10)

    def cb(self, msg: String):
        # We expect JSON in msg.data
        try:
            payload = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f"Invalid JSON payload: {e}")
            return

        if payload.get("event") != "task_allocation":
            # Ignore other events
            return

        # Timestamp (if not provided, use now)
        ts = payload.get("timestamp")
        if not ts:
            ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        task = payload.get("task", {})
        task_id = str(task.get("id", ""))
        task_location = str(task.get("location", ""))

        assigned_robot = str(payload.get("assigned_robot", ""))

        candidates = payload.get("candidates", [])
        if not isinstance(candidates, list) or len(candidates) == 0:
            self.get_logger().warn("No candidates provided in task_allocation event.")
            return

        rows = []
        for c in candidates:
            robot = str(c.get("robot", ""))
            idle = bool(c.get("idle", False))
            workload = int(c.get("workload", 0))

            bat = float(c.get("battery", -1))
            dist = float(c.get("distance", -1))
            score = float(c.get("score", 0.0))

            bat_b = battery_bucket(bat) if bat >= 0 else ""
            dist_b = distance_bucket(dist) if dist >= 0 else ""

            # Round battery to 2 decimal places
            if bat >= 0:
                bat = round(bat, 2)

            is_chosen = (robot == assigned_robot)
            label = 1 if is_chosen else 0

            rows.append([
                ts,
                task_id,
                task_location,
                robot,
                assigned_robot,
                idle,
                workload,
                bat,
                bat_b,
                dist,
                dist_b,
                score,
                is_chosen,
                label
            ])

        try:
            with open(self.csv_path, "a", newline="") as f:
                writer = csv.writer(f)
                writer.writerows(rows)

            self.get_logger().info(
                f"Wrote {len(rows)} ILP rows for task={task_id}, assigned={assigned_robot}"
            )

        except Exception as e:
            self.get_logger().error(f"Failed writing CSV rows: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ILPDatasetLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()