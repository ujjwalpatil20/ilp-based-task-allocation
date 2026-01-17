#!/usr/bin/env python3
"""
battery_sim_node.py

Simulates battery depletion for robots in the warehouse.
"""

import rclpy
from rclpy.node import Node
from robot_interfaces.msg import RobotStatus
from std_msgs.msg import String
import json

class BatterySimNode(Node):
    def __init__(self):
        super().__init__('battery_sim_node')
        
        self.robots = {}
        # Configuration
        self.depletion_rate = 1.0 # percent per tick
        self.charge_rate = 5.0    # percent per tick
        self.tick_rate = 2.0      # seconds
        self.low_threshold = 20.0
        self.full_threshold = 100.0

        # We need to know about robots. For simplicity we'll assume a fixed number or discover them?
        # Better: Subscribe to fleet_status to get robot IDs, but fleet_status needs battery info... circular?
        # Solution: We publish updates, FleetManager subscribes and merges.
        # Initial robots:
        self.robot_ids = [1, 2] # Default, or parameter
        for text_id in self.robot_ids:
            self.robots[text_id] = {
                "battery_level": 100.0,
                "is_charging": False
            }

        self.publisher = self.create_publisher(String, '/simulation/battery_updates', 10)
        self.timer = self.create_timer(self.tick_rate, self.timer_callback)
        self.get_logger().info("Battery Simulation Node Started")

    def timer_callback(self):
        updates = {}
        for robot_id, data in self.robots.items():
            if data["is_charging"]:
                data["battery_level"] += self.charge_rate
                if data["battery_level"] >= self.full_threshold:
                    data["battery_level"] = self.full_threshold
                    data["is_charging"] = False
                    self.get_logger().info(f"Robot {robot_id} fully charged.")
            else:
                data["battery_level"] -= self.depletion_rate
                if data["battery_level"] <= self.low_threshold:
                    data["is_charging"] = True
                    self.get_logger().info(f"Robot {robot_id} battery low ({data['battery_level']}%), charging...")
                elif data["battery_level"] < 0:
                     data["battery_level"] = 0

            updates[robot_id] = data["battery_level"]

        # Publish updates as a JSON string for simplicity, or we could use custom msg
        update_msg = String()
        update_msg.data = json.dumps(updates)
        self.publisher.publish(update_msg)
        # self.get_logger().info(f"Published battery updates: {updates}")

def main(args=None):
    rclpy.init(args=args)
    node = BatterySimNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
