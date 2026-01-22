"""
Task Manager Node

This node is responsible for managing and assigning tasks to a fleet of robots in a warehouse simulation. 
It processes incoming orders, evaluates robot availability, and assigns tasks based on proximity, battery level, and availability. 
The node interacts with other services and topics to ensure efficient task distribution and execution.

### Responsibilities:
1. **Order Management**:
   - Subscribes to the `/order_requests` topic to receive new orders.
   - Hosts `process_order` Action Server to receive orders from Web GUI.
   - Processes orders by assigning tasks to the most suitable robot.
   - Publishes completed orders to the `/end_order` topic.

2. **Task Assignment**:
   - Allocates tasks to robots based on proximity to shelves, battery level, and availability.
   - Creates task messages for robots to execute, including moving to shelves and drop-off locations.

3. **Service Interactions**:
   - Calls the `/get_robot_fleet_status` service to get the status of all robots.
   - Calls the `/get_shelf_list` service to retrieve shelf details.
   - Calls the `/get_drop_off_pose` service to determine the drop-off location.
   - Calls the `/task_assignments` service to assign tasks to robots.

4. **Logging**:
   - Publishes logs to the `/central_logs` topic for centralized logging and monitoring.
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from robot_interfaces.msg import Order, Task, Product, Logs
from robot_interfaces.srv import ShelfQuery, GetRobotStatus, GetRobotFleetStatus, TaskList, GetShelfList, GetPose
from robot_interfaces.action import ProcessOrder
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from geometry_msgs.msg import Pose
from builtin_interfaces.msg import Time

import asyncio
import math
from collections import deque
import threading
import time
import json
from datetime import datetime
from std_msgs.msg import String

from task_management.allocation_strategy import HeuristicAllocator, ILPAllocator

class TaskManager(Node):
    def __init__(self):
        super().__init__('task_manager')

        # Callback group for services (to allow concurrent service calls)
        self.callback_group = ReentrantCallbackGroup()

        # Subscribers
        self.order_subscriber = self.create_subscription(
            Order, '/order_requests', self.order_callback, 10)

        # Publishers
        self.order_end_publisher = self.create_publisher(Order, '/end_order', 10)
        self.log_publisher = self.create_publisher(Logs, '/central_logs', 10)
        
        # ILP Dataset Logger Publisher
        self.ilp_pub = self.create_publisher(String, "/ilp/task_allocation_event", 10)

        # Action Server
        self._action_server = ActionServer(
            self,
            ProcessOrder,
            'process_order',
            self.execute_order_action,
            callback_group=self.callback_group
        )

        # Service clients
        self.shelf_query_client = self.create_client(
            ShelfQuery, '/shelf_query', callback_group=self.callback_group)

        self.robot_fleet_client = self.create_client(
            GetRobotFleetStatus, '/get_robot_fleet_status', callback_group=self.callback_group)

        self.shelf_list_client = self.create_client(
            GetShelfList, '/get_shelf_list', callback_group=self.callback_group)

        self.task_assignment_client = self.create_client(
            TaskList, '/task_list', callback_group=self.callback_group)

        self.get_drop_off_pose_client = self.create_client(
            GetPose, '/get_drop_off_pose', callback_group=self.callback_group)

        # Robot status dictionary
        self.robots = {}  # Format: {robot_id: RobotStatus}

        # Order queue
        self.order_queue = deque()
        self.task_id_counter = 1
        
        # Track action goals to succeed them later
        # Map order_id -> goal_handle
        self.pending_goals = {}
        # Map order_id -> rclpy.task.Future (to unblock executor threads)
        self.order_futures = {}
        
        # Lock to prevent race conditions in robot assignment
        self.assignment_lock = asyncio.Lock()
        
        # Track robots that are currently assigned (to prevent shared_memory stale data issues)
        # Set of robot_ids that we've assigned but haven't finished yet
        self.assigned_robots = set()
        
        # Flag to prevent multiple process_orders coroutines
        self.processing_orders_running = False
        
        # Initialize Allocator Strategy
        # Options: HeuristicAllocator(), ILPAllocator()
        # Initialize Allocator Strategy
        # Options: HeuristicAllocator(), ILPAllocator()
        self.allocator = ILPAllocator(self.get_logger()) # Enabled ILP based on learned rules


        # Start the order processing loop in a separate thread
        self.log_to_central("INFO", "Task Manager is ready.")
        self.loop = asyncio.new_event_loop()
        self.thread = threading.Thread(target=self.run_asyncio_loop, daemon=True)
        self.thread.start()


    def log_to_central(self, level, message):
        """Publishes logs to the central logging topic."""
        log_msg = Logs()
        log_msg.timestamp =  self.get_clock().now().to_msg()
        log_msg.node_name = "Task Manager"
        log_msg.log_level = level
        log_msg.message = message
        self.log_publisher.publish(log_msg)

    def publish_ilp_allocation_event(self, task_id: str, assigned_robot: str, candidates: list,
                                 task_type: str = "order", priority: str = "normal", location: str = ""):
        payload = {
            "event": "task_allocation",
            "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "task": {
                "id": str(task_id),
                "type": str(task_type),
                "priority": str(priority),
                "location": str(location)
            },
            "assigned_robot": str(assigned_robot),
            "candidates": candidates
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.ilp_pub.publish(msg)


    def run_asyncio_loop(self):
        """Run the asyncio event loop in a separate thread."""
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    def create_task(self, coroutine):
        """Schedule an asyncio task in the event loop."""
        asyncio.run_coroutine_threadsafe(coroutine, self.loop)

    def order_callback(self, msg):
        """
        Callback triggered when a new order is received via Topic.
        Adds the order to the queue.
        """
        self.log_to_central("INFO", f'Received order request via Topic: {msg.order_id}')
        self.order_queue.append(msg)
        self.create_task(self.process_orders())  # Trigger order processing

    async def execute_order_action(self, goal_handle):
        """
        Callback for ProcessOrder Action.
        Adds order to queue and waits for completion.
        """
        self.get_logger().info('Executing order action...')
        goal = goal_handle.request
        
        # Create internal Order message from Goal
        order_msg = Order()
        order_msg.order_id = getattr(goal, 'order_id', 0) # Goal might not have it if not set, usually passed
        if order_msg.order_id == 0:
             # Generat ID if needed or assum passed
             import uuid
             order_msg.order_id = int(uuid.uuid4().int >> 64)

        # Map product list
        order_msg.product_list = goal.product_list
        
        # Store handle
        self.pending_goals[order_msg.order_id] = goal_handle
        
        # Create a Future object to wait for completion without blocking the thread
        done_future = rclpy.task.Future()
        self.order_futures[order_msg.order_id] = done_future
        
        self.log_to_central("INFO", f'Received order request via Action: {order_msg.order_id}')
        self.order_queue.append(order_msg)
        
        # Only start process_orders if not already running
        if not self.processing_orders_running:
            self.create_task(self.process_orders())
        
        # Monitor status
        feedback_msg = ProcessOrder.Feedback()
        feedback_msg.order_id = order_msg.order_id
        feedback_msg.status = "Queued"
        goal_handle.publish_feedback(feedback_msg)
        
        # Await the future. This yields the executor thread to other tasks.
        result = await done_future
        
        del self.order_futures[order_msg.order_id]
        return result


    async def process_orders(self):
        """
        Continuously processes orders from the queue in strict FIFO order.
        Only one order is processed at a time per robot.
        """
        self.processing_orders_running = True
        last_logged_no_robot_order_id = None
        try:
            while rclpy.ok():
                if self.order_queue:
                    # Peek at the first order (don't remove yet)
                    order = self.order_queue[0]
                    if order.order_id != last_logged_no_robot_order_id:
                         self.log_to_central("INFO", f'Processing order: {order.order_id}')
                         self.get_logger().info(f'Processing order: {order.order_id}')
                    
                    # Try to find an available robot
                    robot_fleet_response, shelf_query_response, drop_off_pose = await asyncio.gather(
                        self.call_robot_fleet_service(),
                        self.call_shelf_list_service(),
                        self.get_drop_off_pose()
                    )
                    
                    task_assigned, last_logged_no_robot_order_id = self.allocate_task(robot_fleet_response, shelf_query_response, drop_off_pose, order, last_logged_no_robot_order_id)
                    
                    if task_assigned:
                        # Robot found! Remove order from queue and process it
                        self.order_queue.popleft()
                        assigned_robot_id = task_assigned[0].robot_id
                        
                        # Assign the task
                        success = await self.call_task_assignment_service(task_assigned)
                        
                        if success:
                            # Mark robot as assigned locally
                            self.assigned_robots.add(assigned_robot_id)
                            
                            # Spawn a separate coroutine to wait for this order's completion
                            total_tasks = len(task_assigned)
                            asyncio.create_task(self.wait_for_order_completion(order, assigned_robot_id, total_tasks))
                        else:
                            # Task assignment failed - put order back at front
                            self.order_queue.appendleft(order)
                            self.log_to_central("WARN", "Task assignment failed, requeueing order.")
                            await asyncio.sleep(2.0)
                    else:
                        # No robot available - send feedback and wait
                        if order.order_id in self.pending_goals:
                            fb = ProcessOrder.Feedback()
                            fb.order_id = order.order_id
                            fb.status = "Queued: Waiting for Robot"
                            fb.progress = 0.0
                            self.pending_goals[order.order_id].publish_feedback(fb)
                        
                        # Wait before checking again
                        await asyncio.sleep(2.0)
                else:
                    # Queue empty, exit this coroutine. Will be re-triggered by next order.
                    return
        except Exception as e:
             import traceback
             self.log_to_central("ERROR", f"process_orders loop crashed: {e}\n{traceback.format_exc()}")
        finally:
            self.processing_orders_running = False
    
    async def wait_for_order_completion(self, order, assigned_robot_id, total_tasks):
        """
        Waits for a robot to finish navigating and completes the order.
        Runs as a separate coroutine so queue processing can continue.
        """
        try:
            # Feedback: Robot assigned, navigation started
            if order.order_id in self.pending_goals:
                fb = ProcessOrder.Feedback()
                fb.order_id = order.order_id
                fb.status = f"Navigating (Robot {assigned_robot_id})"
                fb.progress = 0.1
                self.pending_goals[order.order_id].publish_feedback(fb)
            
            # --- Fix for Race Condition: Wait for Robot to become BUSY first ---
            # The robot might still be 'idle' from previous state for a split second before FleetManager updates it.
            # We must verify it transitions to 'not available' before we start checking for it to become 'available' (idle) again.
            busy_wait_start = self.get_clock().now()
            is_robot_busy = False
            while (self.get_clock().now() - busy_wait_start).nanoseconds / 1e9 < 10.0: # 10s timeout
                fleet_status = await self.call_robot_fleet_service()
                robot_status = next(
                    (r for r in fleet_status.robot_status_list if r.robot_id == assigned_robot_id), 
                    None
                )
                if robot_status and not robot_status.is_available:
                    is_robot_busy = True
                    self.log_to_central("INFO", f"Robot {assigned_robot_id} confirmed busy for Order {order.order_id}")
                    break
                await asyncio.sleep(0.2)
            
            if not is_robot_busy:
                self.log_to_central("WARN", f"Robot {assigned_robot_id} never became busy for Order {order.order_id}. Assuming assignment failure.")
                self.assigned_robots.discard(assigned_robot_id)
                if order.order_id in self.pending_goals:
                    self.pending_goals[order.order_id].abort()
                if order.order_id in self.order_futures:
                    self.order_futures[order.order_id].set_result(ProcessOrder.Result(success=False, message="Robot failed to start"))
                return

            # Wait for robot to finish navigation (poll until idle)
            max_wait = 300  # 5 minutes max
            wait_time = 0
            success = False
            
            while wait_time < max_wait:
                await asyncio.sleep(0.5) # Fast polling for smoother UI
                wait_time += 0.5
                
                # Check robot status
                fleet_status = await self.call_robot_fleet_service()
                robot_status = next(
                    (r for r in fleet_status.robot_status_list if r.robot_id == assigned_robot_id), 
                    None
                )
                
                if robot_status and robot_status.is_available:
                    # Robot finished!
                    self.order_end_publisher.publish(order)
                    self.log_to_central("INFO", f"Order {order.order_id} Completed Successfully.")
                    
                    if order.order_id in self.pending_goals:
                        fb = ProcessOrder.Feedback()
                        fb.order_id = order.order_id
                        fb.status = "Completed"
                        fb.progress = 1.0
                        self.pending_goals[order.order_id].publish_feedback(fb)
                    
                    success = True
                    break
                
                # Update progress feedback periodically based on robot status
                progress = 0.1
                if robot_status and "Busy: Waypoint" in robot_status.status:
                    try:
                        # Status format: "Busy: Waypoint X"
                        parts = robot_status.status.split()
                        if len(parts) >= 3:
                            completed_waypoints = int(parts[2])
                            if total_tasks > 0:
                                # Add 0.5 to show we are "in progress" of the current leg
                                progress = 0.1 + ((completed_waypoints + 0.5) / total_tasks) * 0.8
                                progress = min(progress, 0.99)
                    except ValueError:
                        pass

                if order.order_id in self.pending_goals:
                    fb = ProcessOrder.Feedback()
                    fb.order_id = order.order_id
                    fb.status = f"Navigating (Robot {assigned_robot_id})"
                    fb.progress = progress
                    self.pending_goals[order.order_id].publish_feedback(fb)
            
            # Remove from local tracking
            self.assigned_robots.discard(assigned_robot_id)
            
            # Notify allocator of completion (for workload tracking etc)
            self.allocator.on_task_completion(assigned_robot_id)
            
            # Complete the action
            result_msg = None
            if order.order_id in self.pending_goals:
                if success:
                    self.pending_goals[order.order_id].succeed()
                    result_msg = ProcessOrder.Result(success=True, message="Order Processed")
                else:
                    self.log_to_central("WARN", f"Order {order.order_id} timed out waiting for robot.")
                    self.pending_goals[order.order_id].abort()
                    result_msg = ProcessOrder.Result(success=False, message="Timed Out")
                del self.pending_goals[order.order_id]
            
            # Unblock the execute callback for this order
            if order.order_id in self.order_futures:
                 self.order_futures[order.order_id].set_result(result_msg)

        except Exception as e:
            import traceback
            self.log_to_central("ERROR", f"wait_for_order_completion crashed for Order {order.order_id}: {e}\n{traceback.format_exc()}")
            self.assigned_robots.discard(assigned_robot_id)
            if order.order_id in self.order_futures and not self.order_futures[order.order_id].done():
                self.order_futures[order.order_id].set_result(ProcessOrder.Result(success=False, message=f"Internal Error: {e}"))


    async def call_robot_fleet_service(self):
        """
        Call the /get_robot_fleet_status service asynchronously.
        """
        while not self.robot_fleet_client.wait_for_service(timeout_sec=1.0):
            self.log_to_central("WARN", 'Waiting for /get_robot_fleet_status service...')

        request = GetRobotFleetStatus.Request()
        future = self.robot_fleet_client.call_async(request)
        await future
        return future.result()

    async def call_shelf_list_service(self):
        """
        Call the /shelf_query service asynchronously.
        """
        while not self.shelf_list_client.wait_for_service(timeout_sec=1.0):
            self.log_to_central("WARN", 'Waiting for /shelf_query service...')

        # returns a list of shelf details
        request = GetShelfList.Request()
        future = self.shelf_list_client.call_async(request)
        await future
        return future.result()

    async def get_drop_off_pose(self):
        """
        Get the drop_off_pose from the shared_memory_node.
        """
        while not self.get_drop_off_pose_client.wait_for_service(timeout_sec=1.0):
            self.log_to_central("WARN", 'Waiting for get_drop_off_pose service...')

        request = GetPose.Request()
        future = self.get_drop_off_pose_client.call_async(request)

        await future
        return future.result()

    def allocate_task(self, robot_fleet_response, shelf_query_response, drop_off_pose, order, last_logged_no_robot_order_id):
        # Sort the order's product list based on shelf_id
        order.product_list.sort(key=lambda product: product.shelf_id)
        
        # Use strategy to select robot
        robot_fleet_list = robot_fleet_response.robot_status_list
        shelf_details = shelf_query_response.shelf_status_list
        
        best_robot_id, log_msg, log_level, candidates = self.allocator.select_best_robot(
            robot_fleet_list, shelf_details, order, self.assigned_robots
        )
        
        if log_msg:
             # Only log warning if it's new
             if not (log_level == "WARN" and order.order_id == last_logged_no_robot_order_id):
                 self.log_to_central(log_level or "INFO", log_msg)

        if best_robot_id:
            # Create a Task message for each product in the order
            task_list = []
            for product in order.product_list:
                shelf = next((s for s in shelf_details if s.shelf_id == product.shelf_id), None)
                if not shelf: continue
                
                task_msg = Task()
                task_msg.task_id = self.task_id_counter
                task_msg.robot_id = best_robot_id
                task_msg.shelf_id = product.shelf_id
                task_msg.shelf_location = shelf.shelf_location
                task_msg.item = shelf.product
                task_msg.item_amount = product.quantity
                task_msg.task_type = f"Move to Shelf {product.shelf_id}"
                self.task_id_counter += 1
                task_list.append(task_msg)

            drop_off_task = self.get_drop_off_task(best_robot_id, drop_off_pose)
            task_list.append(drop_off_task)
            self.log_to_central("INFO", f"Assigned Order to robot {best_robot_id}: {len(task_list)} tasks")
            self.get_logger().info(f"Assigned Order to robot {best_robot_id}: {len(task_list)} tasks")

            task_id_log = f"order_{order.order_id}"
            loc_log = str(order.product_list[0].shelf_id) if order.product_list else ""
            self.publish_ilp_allocation_event(
                task_id=task_id_log,
                assigned_robot=str(best_robot_id),
                candidates=candidates,
                task_type="order_pick",
                location=loc_log
            )

            return task_list, last_logged_no_robot_order_id
        else:
            if order.order_id != last_logged_no_robot_order_id:
                 last_logged_no_robot_order_id = order.order_id
            return None, last_logged_no_robot_order_id


    def get_drop_off_task(self, robot_id, drop_off_pose):
        # Calculate specific drop-off location for the robot
        drop_off_x_offset = 0.6  # 60 cm offset
        specific_drop_off_pose = Pose()
        specific_drop_off_pose.position.x = drop_off_pose.pose.position.x + robot_id * drop_off_x_offset
        specific_drop_off_pose.position.y = drop_off_pose.pose.position.y
        specific_drop_off_pose.position.z = drop_off_pose.pose.position.z
        specific_drop_off_pose.orientation = drop_off_pose.pose.orientation

        # Add a final task to move to the specific drop-off location
        drop_off_task = Task()
        drop_off_task.task_id = self.task_id_counter
        drop_off_task.robot_id = robot_id
        drop_off_task.shelf_id = 0
        drop_off_task.shelf_location = specific_drop_off_pose
        drop_off_task.item = ""
        drop_off_task.item_amount = 0
        drop_off_task.task_type = "Move to Drop-off"
        self.task_id_counter += 1
        return drop_off_task

    async def call_task_assignment_service(self, task_list):
        """
        Call the /task_assignments service to assign the task to the robot.
        Returns True if the task was successfully assigned, False otherwise.
        """
        while not self.task_assignment_client.wait_for_service(timeout_sec=1.0):
            self.log_to_central("WARN", 'Waiting for /task_list service...')

        request = TaskList.Request()
        request.task_list = task_list
        future = self.task_assignment_client.call_async(request)
        await future
        return future.result().success

    def calculate_distance(self, location1: Pose, location2: Pose):
        """
        Calculates the distance between two locations.
        """
        return math.sqrt(
            (location2.position.x - location1.position.x) ** 2 + (location2.position.y - location1.position.y) ** 2)


def main(args=None):
    rclpy.init(args=args)

    # Create the node
    task_manager = TaskManager()

    # Use MultiThreadedExecutor
    executor = MultiThreadedExecutor()

    try:
        # Spin the node
        rclpy.spin(task_manager, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        task_manager.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()