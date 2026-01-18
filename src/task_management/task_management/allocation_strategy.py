import math
from geometry_msgs.msg import Pose

class AllocationStrategy:
    """
    Base class for robot allocation strategies.
    """
    def select_best_robot(self, robot_fleet_list, shelf_details, order, assigned_robots):
        """
        Determines the best robot for a given order.
        
        Args:
            robot_fleet_list (list): List of RobotStatus objects.
            shelf_details (list): List of ShelfStatus objects.
            order (Order): The order to process.
            assigned_robots (set): Set of robot IDs that are currently assigned.
            
        Returns:
            tuple: (best_robot_id (int or None), log_message (str or None), log_level (str or None), candidates (list))
        """
        raise NotImplementedError("Subclasses must implement select_best_robot")


class HeuristicAllocator(AllocationStrategy):
    """
    Allocates tasks to robots based on proximity to shelves and battery level.
    """
    def select_best_robot(self, robot_fleet_list, shelf_details, order, assigned_robots):
        best_robot_id = None
        best_score = -1
        
        # Ensure we have products to check
        if not order.product_list:
             return None, "Order has no products.", "WARN"

        # Use the first product's shelf for distance calculation
        # Note: The order's product list should be sorted by preference/shelf_id before calling this, 
        # or we assume the first one is the target for heuristic.
        first_product_shelf_id = order.product_list[0].shelf_id
        target_shelf = next((s for s in shelf_details if s.shelf_id == first_product_shelf_id), None)

        if not target_shelf:
            return None, f"Shelf {first_product_shelf_id} not found.", "WARN"

        shelf_location = target_shelf.shelf_location

        found_candidate = False
        
        for robot in robot_fleet_list:
            # Check both shared_memory status AND local tracking
            if robot.is_available and robot.robot_id not in assigned_robots:
                found_candidate = True
                robot_location = robot.current_location
                distance = self._calculate_distance(robot_location, shelf_location)
                battery_score = robot.battery_level
                
                # Combined score (lower distance and higher battery are better)
                # Adding 1 to distance to avoid division by zero
                score = (1 / (distance + 1)) * battery_score

                if score > best_score:
                    best_score = score
                    best_robot_id = robot.robot_id
        
        if best_robot_id is None:
             if found_candidate:
                 # Robots were available but somehow score didn't pick one? 
                 # With logic above, this theoretically shouldn't happen if found_candidate is true 
                 # unless best_score stays -1 (which it starts at).
                 # score will always be > 0.
                 pass
             return None, "No available robots to assign task.", "WARN", []
             
        return best_robot_id, None, None, []

    def _calculate_distance(self, location1: Pose, location2: Pose):
        return math.sqrt(
            (location2.position.x - location1.position.x) ** 2 + 
            (location2.position.y - location1.position.y) ** 2
        )

        
class ILPAllocator(AllocationStrategy):
    """
    Allocates tasks using a 3-step hybrid logic (Overwork check, Battery check, Proximity).
    Tracks workload to prevent over-assigning to the same robot.
    """
    def __init__(self):
        from collections import defaultdict
        self.robot_workload = defaultdict(int)

    def select_best_robot(self, robot_fleet_list, shelf_details, order, assigned_robots):
        best_robot_id = None
        best_score = -1
        
        candidates = [] # For logging if needed, or return it?
        
        # Ensure we have products
        if not order.product_list:
             return None, "Order has no products.", "WARN"

        first_product_shelf_id = order.product_list[0].shelf_id
        target_shelf = next((s for s in shelf_details if s.shelf_id == first_product_shelf_id), None)
        
        if not target_shelf:
            return None, f"Shelf {first_product_shelf_id} not found.", "WARN"
            
        shelf_location = target_shelf.shelf_location

        found_candidate = False

        for robot in robot_fleet_list:
            if robot.is_available and robot.robot_id not in assigned_robots:
                found_candidate = True
                robot_location = robot.current_location
                dist = self._calculate_distance(robot_location, shelf_location)
                
                current_workload = self.robot_workload[robot.robot_id]
                
                # --- 3-Step Hybrid Logic ---
                
                # 1. Overworked? (Workload > 2)
                if current_workload > 2:
                    # self.log_to_central? No access to logger here unless passed.
                    # We'll just set low score.
                    score = -1.0 
                    
                # 2. Critical Battery? (<= 40%) - Survival Mode
                elif robot.battery_level <= 0.40:
                    # Prioritize Most Charged (Score = Battery Level * 10)
                    score = float(robot.battery_level) * 10.0
                    
                # 3. Normal Operation - Efficiency Mode
                else:
                    # Prioritize Closest (Score = 100 / (Distance + 1))
                    score = 100.0 / (dist + 1.0)

                if score > best_score:
                    best_score = score
                    best_robot_id = robot.robot_id

                candidates.append({
                    "robot": robot.robot_id,
                    "battery": float(robot.battery_level),
                    "idle": bool(robot.is_available),
                    "workload": int(current_workload),
                    "distance": float(dist)
                })
        
        if best_robot_id:
            # Increment workload
            self.robot_workload[best_robot_id] += 1
            return best_robot_id, None, None, candidates
        
        return None, "No available robots to assign task.", "WARN", candidates

    def on_task_completion(self, robot_id):
        if self.robot_workload[robot_id] > 0:
            self.robot_workload[robot_id] -= 1

    def _calculate_distance(self, location1: Pose, location2: Pose):
        return math.sqrt(
            (location2.position.x - location1.position.x) ** 2 + 
            (location2.position.y - location1.position.y) ** 2
        )
