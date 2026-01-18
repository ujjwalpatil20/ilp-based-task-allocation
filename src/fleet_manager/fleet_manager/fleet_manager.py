"""
fleet_manager.py

This module implements the FleetManager class, which is responsible for managing a fleet of robots in a multi-robot task distribution system. 
The FleetManager node handles robot status, task assignment, and communication with robots using ROS 2.

Classes:
    FleetManager(Node): A ROS 2 node that manages a fleet of robots, including their status, task assignments, and communication.

Functions:
    main(args=None): Initializes the FleetManager node and starts the ROS 2 event loop.

FleetManager Class:
    Methods:
        __init__(num_robots): Initializes the FleetManager node, sets up robot data structures, subscriptions, publishers, and services.
        log_to_central(level, message, robot_namespace=None, log_source="FleetManager"): Publishes logs to a central logging topic.
        odom_callback(msg, robot_name): Updates the location of a robot based on Odometry messages.
        publish_fleet_status(): Publishes the current status of the fleet to the 'fleet_status' topic.
        handle_task_list(request, response): Handles incoming task list service requests and assigns tasks to robots.
        create_waypoints(task_list): Converts a list of tasks into waypoints for robot navigation.
        send_goal(robot_namespace, waypoints): Sends a navigation goal to a robot using the FollowWaypoints action.
        feedback_callback(feedback_msg, goal_id): Handles feedback from the robot during task execution.
        goal_response_callback(future, robot_namespace, client): Handles the response to a goal request, updating robot availability.
        get_result_callback(future, robot_namespace): Processes the result of a completed goal and updates robot status.

Usage:
    This script is intended to be run as a ROS 2 node. It manages a fleet of robots, assigns tasks, and monitors their progress.
    To run the node, execute the script directly:
        $ python3 fleet_manager.py

Dependencies:
    - ROS 2 (rclpy)
    - nav_msgs.msg (Odometry)
    - geometry_msgs.msg (Pose, PoseStamped)
    - nav2_msgs.action (FollowWaypoints)
    - robot_interfaces.msg (RobotStatus, Task, FleetStatus)
    - robot_interfaces.srv (TaskList)
    - std_msgs.msg (String)
    - uuid (for generating unique goal IDs)

Parameters:
    - num_robots (int): The number of robots in the fleet. This parameter is retrieved from the ROS 2 parameter server.

Topics:
    - /central_logs (String): Publishes logs for centralized monitoring.
    - /<robot_name>/odom (Odometry): Subscribes to robot odometry data.
    - fleet_status (FleetStatus): Publishes the status of the fleet.

Services:
    - task_list (TaskList): Handles task assignment requests.

Actions:
    - /<robot_name>/follow_waypoints (FollowWaypoints): Sends navigation goals to robots.

Example:
    To run the FleetManager node with 3 robots:
        1. Set the 'num_robots' parameter to 3 in the ROS 2 parameter server.
        2. Run the script:
            $ python3 fleet_manager.py
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseStamped
from nav2_msgs.action import FollowWaypoints
from robot_interfaces.msg import RobotStatus, Task, FleetStatus, Logs
from robot_interfaces.srv import TaskList, GetShelfList, GetPose
from robot_interfaces.action import ProcessOrder
import uuid
import json
from std_msgs.msg import String
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from builtin_interfaces.msg import Time

class FleetManager(Node):
    def __init__(self):
        """Initializes the FleetManager node and sets up robots, publishers, and services."""
        super().__init__('fleet_manager')

        self.declare_parameter('num_robots', 2)
        num_robots = self.get_parameter('num_robots').value
        self.get_logger().info(f"Initialized FleetManager with {num_robots} robots.")
       
        self.log_publisher = self.create_publisher(Logs, '/central_logs', 10)

        self.robots = {}
        self.goal_handles = {}
        self.previous_feedback = {}
        self.robot_to_goal_id = {}

        # Initialize robots and subscriptions
        for i in range(1, num_robots + 1):
            robot_name = f'robot_{i}'
            odom_topic = f'/{robot_name}/odom'
            self.robots[robot_name] = {
                "id": i,
                "location": Pose(),
                "battery_level": 100.0,
                "is_available": True,
                "status": "idle"
            }
            self.create_subscription(Odometry, odom_topic, lambda msg, name=robot_name: self.odom_callback(msg, name), 10)

        # Create a QoS profile for fleet status
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        # Create a publisher for fleet status with the QoS profile
        self.fleet_status_publisher = self.create_publisher(FleetStatus, 'fleet_status', qos_profile)

        # Create a timer to periodically publish fleet status
        self.timer = self.create_timer(1.0, self.publish_fleet_status)

        # Battery Subscription
        self.create_subscription(String, '/simulation/battery_updates', self.battery_callback, 10)

        # Client to query shelf locations
        self.shelf_list_client = self.create_client(GetShelfList, '/get_shelf_list')
        self.get_drop_off_pose_client = self.create_client(GetPose, '/get_drop_off_pose')

        # Service Server for Task Assignment
        self.create_service(TaskList, 'task_list', self.handle_task_list)

    # execute_order_callback removed as logic is moved to task_manager

    def battery_callback(self, msg):
        try:
            updates = json.loads(msg.data)
            for robot_id, level in updates.items():
                robot_name = f'robot_{robot_id}'
                if robot_name in self.robots:
                    self.robots[robot_name]["battery_level"] = float(level)
        except Exception as e:
            self.get_logger().error(f"Failed to parse battery update: {e}")

    def log_to_central(self, level, message, robot_namespace=None, log_source="FleetManager"):
        """Publishes logs to the central logging topic."""
        log_msg = Logs()
        log_msg.timestamp =  self.get_clock().now().to_msg()
        log_msg.node_name = "Fleet Manager"
        log_msg.log_level = level
        log_msg.message = message
        self.log_publisher.publish(log_msg)
        
    def odom_callback(self, msg, robot_name):
        """Updates the location of a robot based on Odometry messages."""
        if robot_name in self.robots:
            self.robots[robot_name]["location"] = msg.pose.pose

    def publish_fleet_status(self):
        """Publishes the current status of the fleet to the 'fleet_status' topic."""
        fleet_status_msg = FleetStatus()
        fleet_status_msg.robot_status_list = []
        for robot in self.robots.values():
            robot_status = RobotStatus()
            robot_status.robot_id = robot["id"]
            robot_status.current_location = robot["location"]
            robot_status.battery_level = robot["battery_level"]
            robot_status.is_available = robot["is_available"]
            robot_status.status = robot["status"]
            fleet_status_msg.robot_status_list.append(robot_status)
        self.fleet_status_publisher.publish(fleet_status_msg)

    def handle_task_list(self, request, response):
        """Handles incoming task list service requests and assigns tasks to robots."""
        try:
            if request.task_list:
                robot_id = request.task_list[0].robot_id
                robot_namespace = f'robot_{robot_id}'
                if not self.robots[robot_namespace]["is_available"]:
                    response.success = False
                    return response

                waypoints = self.create_waypoints(request.task_list)
                self.robots[robot_namespace]["is_available"] = False
                self.robots[robot_namespace]["status"] = "busy"
                self.send_goal(robot_namespace, waypoints)
                response.success = True
                task_count = len(request.task_list)
                self.log_to_central('INFO', f'received {task_count} tasks for {robot_namespace}.', robot_namespace, "handle_task_list")
            else:
                self.log_to_central('INFO', 'No tasks available', log_source="handle_task_list")
                response.success = False
        except Exception as e:
            self.log_to_central('ERROR', f'Service call failed: {e}', log_source="handle_task_list")
            response.success = False
        return response

    def create_waypoints(self, task_list):
        """Converts a list of tasks into waypoints for robot navigation."""
        waypoints = []
        for task in task_list:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose = task.shelf_location
            waypoints.append(pose)
        return waypoints

    def send_goal(self, robot_namespace, waypoints):
        """Sends a navigation goal to a robot using the FollowWaypoints action."""
        client = ActionClient(self, FollowWaypoints, f'/{robot_namespace}/follow_waypoints')
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = waypoints
        client.wait_for_server()

        goal_uuid = uuid.uuid4()
        self.robot_to_goal_id[str(goal_uuid)] = robot_namespace

        send_goal_future = client.send_goal_async(
            goal_msg,
            feedback_callback=lambda feedback_msg, goal_id=str(goal_uuid): self.feedback_callback(feedback_msg, goal_id)
        )
        send_goal_future.add_done_callback(lambda future: self.goal_response_callback(future, robot_namespace, client))

    def feedback_callback(self, feedback_msg, goal_id):
        """Handles feedback from the robot during task execution."""
        feedback = feedback_msg.feedback
        feedback_str = str(feedback)

        robot_namespace = self.robot_to_goal_id.get(goal_id)
        if not robot_namespace:
            self.log_to_central('ERROR', f'No robot namespace found for goal ID: {goal_id}', log_source="feedback_callback")
            return

        if robot_namespace not in self.previous_feedback:
            self.previous_feedback[robot_namespace] = None

        if feedback_str != self.previous_feedback[robot_namespace]:
            # self.log_to_central('INFO', f'Robot {robot_namespace}: Feedback changed: {feedback_str}', robot_namespace, "feedback_callback")
            if hasattr(feedback, 'current_waypoint'):
                 self.log_to_central('INFO', f'Robot {robot_namespace} reached waypoint {feedback.current_waypoint}', robot_namespace, "feedback_callback")
                 self.robots[robot_namespace]["status"] = f"Busy: Waypoint {feedback.current_waypoint}"
            self.previous_feedback[robot_namespace] = feedback_str

    def goal_response_callback(self, future, robot_namespace, client):
        """Handles the response to a goal request, updating robot availability."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.log_to_central('INFO', f'Goal rejected for {robot_namespace}', robot_namespace, "goal_response_callback")
            self.robots[robot_namespace]["is_available"] = True
            self.robots[robot_namespace]["status"] = "idle"
            return

        self.log_to_central('INFO', f'Goal accepted for {robot_namespace}', robot_namespace, "goal_response_callback")
        self.goal_handles[robot_namespace] = goal_handle
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(lambda future: self.get_result_callback(future, robot_namespace))

    def get_result_callback(self, future, robot_namespace):
        """Processes the result of a completed goal and updates robot status."""
        try:
            result = future.result()
            status = result.status
            # Status 4 = SUCCEEDED, 5 = CANCELED, 6 = ABORTED
            if status == 4:
                self.log_to_central('INFO', f'Navigation completed for {robot_namespace}', robot_namespace, "get_result_callback")
            else:
                self.log_to_central('WARN', f'Navigation ended with status {status} for {robot_namespace}', robot_namespace, "get_result_callback")
        except Exception as e:
            self.log_to_central('ERROR', f'Failed to get result for {robot_namespace}: {e}', robot_namespace, "get_result_callback")
        
        # Set robot back to available regardless of success/failure
        self.robots[robot_namespace]["is_available"] = True
        self.robots[robot_namespace]["status"] = "idle"
        self.log_to_central('INFO', f'Robot {robot_namespace} is now idle', robot_namespace, "get_result_callback")

def main(args=None):
    """Initializes the FleetManager node and starts the ROS 2 event loop."""
    rclpy.init(args=args)
    
    # Create a temporary node to declare and retrieve parameters
    # temp_node = rclpy.create_node('temp_node')
    # temp_node.declare_parameter('num_robots', 2)  # Default value is 2
    # num_robots = temp_node.get_parameter('num_robots').value
    # temp_node.destroy_node()
    
    fleet_manager = FleetManager()
    executor = MultiThreadedExecutor()
    rclpy.spin(fleet_manager, executor=executor)
    fleet_manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()