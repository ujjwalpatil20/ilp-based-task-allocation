#!/usr/bin/env python3
"""
battery_sim_node.py

Simulates battery depletion for robots in the warehouse.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import random

class BatterySimNode(Node):
    def __init__(self):
        super().__init__('battery_sim_node')
        
        self.declare_parameter('num_robots', 2)
        self.num_robots = self.get_parameter('num_robots').get_parameter_value().integer_value
        
        self.robots = {}
        # Configuration
        self.depletion_rate = 1.0 # percent per tick
        self.charge_rate = 4.0    # percent per tick
        self.tick_rate = 1.0      # seconds
        self.low_threshold = 10.0
        self.full_threshold = 100.0

        # Dynamic robot initialization
        # Assuming robot IDs are integers 1..N based on 'robot_N' naming convention
        self.robot_ids = list(range(1, self.num_robots + 1))
        
        for robot_id in self.robot_ids:
            # Start with random battery percentage between 50% and 100%
            initial_battery = random.uniform(50.0, 100.0)
            self.robots[str(robot_id)] = {
                "battery_level": initial_battery,
                "is_charging": False
            }

        self.publisher = self.create_publisher(String, '/simulation/battery_updates', 10)
        self.timer = self.create_timer(self.tick_rate, self.timer_callback)
        self.get_logger().info(f"Battery Simulation Node Started for {self.num_robots} robots with random levels.")

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
