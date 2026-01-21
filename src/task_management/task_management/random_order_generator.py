#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from robot_interfaces.msg import Order, Product
from robot_interfaces.srv import GetShelfList
import random
import time
import argparse
import uuid

class RandomOrderGenerator(Node):
    def __init__(self, num_orders):
        super().__init__('random_order_generator')
        self.num_orders = num_orders
        
        # Publishers and Clients
        self.order_publisher = self.create_publisher(Order, '/order_requests', 10)
        self.shelf_list_client = self.create_client(GetShelfList, '/get_shelf_list')
        
        # Wait for shelf list service
        self.get_logger().info('Waiting for shelf service...')
        while not self.shelf_list_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Shelf service not available, waiting again...')
            
        self.available_shelves = []
        self.fetch_shelves()

    def fetch_shelves(self):
        req = GetShelfList.Request()
        future = self.shelf_list_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response:
            self.available_shelves = [s.shelf_id for s in response.shelf_status_list if s.current_inventory > 0]
            self.get_logger().info(f"Found {len(self.available_shelves)} available shelves")
        else:
            self.get_logger().error("Failed to fetch shelf list")

    def run(self):
        if not self.available_shelves:
            self.get_logger().error("No shelves available to order from.")
            return

        for i in range(self.num_orders):
            order = Order()
            # Generate a random 32-bit int ID
            order.order_id = random.randint(100, 999)
            
            # Random number of items (1 to 3)
            num_items = random.randint(1, 3)
            selected_shelves = random.sample(self.available_shelves, min(num_items, len(self.available_shelves)))
            
            for shelf_id in selected_shelves:
                p = Product()
                p.shelf_id = shelf_id
                p.quantity = random.randint(1, 3)
                order.product_list.append(p)
            
            self.get_logger().info(f"Publishing Order {i+1}/{self.num_orders}: ID={order.order_id} Items={len(order.product_list)}")
            self.order_publisher.publish(order)
            
            # Random sleep 0.5 to 2.0 seconds
            sleep_time = random.uniform(0.5, 2.0)
            time.sleep(sleep_time)

def main(args=None):
    rclpy.init(args=args)
    
    parser = argparse.ArgumentParser(description='Generate random orders')
    parser.add_argument('n', type=int, help='Number of orders to generate')
    
    # We need to filter out ROS args from sys.argv for argparse
    # But rclpy.init handles them. Simple hack: handle our arg manually or use index
    # Standard argparse is tricky with ROS2 args mixed in.
    # Let's assume the user passes N as the first non-ros arg.
    
    import sys
    # Filter out potential ros args if mixed, though normally ros2 run handles this.
    # We will just parse known args.
    
    try:
        # A simple workaround for ros2 run arguments
        cleaned_args = [arg for arg in sys.argv[1:] if not arg.startswith('--ros-args')]
        if not cleaned_args:
             print("Usage: ros2 run task_management random_order_generator <n>")
             return
        
        n = int(cleaned_args[0])
    except ValueError:
        print("Please provide an integer for N")
        return

    node = RandomOrderGenerator(n)
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
