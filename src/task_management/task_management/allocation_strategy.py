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

    def on_task_completion(self, robot_id):
        """
        Callback when a task is completed.
        """
        pass


class HeuristicAllocator(AllocationStrategy):
    """
    Allocates tasks to robots based on proximity to shelves and battery level.
    """
    def __init__(self, logger=None):
        self.logger = logger

    def select_best_robot(self, robot_fleet_list, shelf_details, order, assigned_robots):
        best_robot_id = None
        best_score = -1
        candidates = []
        
        # Ensure we have products to check
        if not order.product_list:
             return None, "Order has no products.", "WARN", []

        # Use the first product's shelf for distance calculation
        # Note: The order's product list should be sorted by preference/shelf_id before calling this.
        first_product_shelf_id = order.product_list[0].shelf_id
        target_shelf = next((s for s in shelf_details if s.shelf_id == first_product_shelf_id), None)

        if not target_shelf:
            return None, f"Shelf {first_product_shelf_id} not found.", "WARN", []

        shelf_location = target_shelf.shelf_location

        found_candidate = False
        
        for robot in robot_fleet_list:
            # Determine effective availability:
            # Must be available in FleetManager AND not locked locally
            is_effectively_idle = robot.is_available and (robot.robot_id not in assigned_robots)

            # Calculate metrics for all active robots (for logging stats)
            robot_location = robot.current_location
            distance = self._calculate_distance(robot_location, shelf_location)
            battery_score = float(robot.battery_level)
            
            # Calculate heuristic score for all candidates
            # (1 / (distance + 1)) * battery_score
            score = (1 / (distance + 1)) * battery_score

            # Bucket logic (matching ILPAllocator for consistent dataset)
            bat_b = "high"
            if battery_score < 0.30: bat_b = "low"
            elif battery_score < 0.60: bat_b = "medium"
            
            dist_b = "far"
            if distance <= 3.0: dist_b = "near"
            elif distance <= 7.0: dist_b = "medium"
            
            # Heuristic currently doesn't track workload history, default to zero
            wl_b = "zero" 
            workload = 0
            
            candidates.append({
                "robot": robot.robot_id,
                "battery": battery_score,
                "distance": distance,
                "workload": workload,
                "idle": is_effectively_idle,
                "battery_bucket": bat_b,
                "distance_bucket": dist_b,
                "workload_bucket": wl_b,
                "score": score
            })

            # Check for availability for actual assignment
            if is_effectively_idle:
                found_candidate = True
                
                if score > best_score:
                    best_score = score
                    best_robot_id = robot.robot_id
        
        if best_robot_id is None:
             if found_candidate:
                 # Robots were available but somehow score didn't pick one? 
                 pass
             return None, "No available robots to assign task.", "WARN", candidates
             
        return best_robot_id, None, None, candidates

    def on_task_completion(self, robot_id):
        pass

    def _calculate_distance(self, location1: Pose, location2: Pose):
        return math.sqrt(
            (location2.position.x - location1.position.x) ** 2 + 
            (location2.position.y - location1.position.y) ** 2
        )

        
class ILPAllocator(AllocationStrategy):
    """
    Allocates tasks using rules learned via ILP (Inductive Logic Programming).
    Robustly falls back to HeuristicAllocator if rules are missing or fail.
    """
    def __init__(self, logger=None):
        from collections import defaultdict
        import os
        
        self.logger = logger
        self.robot_workload = defaultdict(int)
        self.use_ilp = False
        self.janus = None
        
        # --- 1. Attempt to Load Janus and Rules ---
        try:
             import janus_swi as janus
             self.janus = janus
             
             # Locate rules file
             # Priority 1: explicitly passed path (not using this ctor arg for now to keep it simple)
             # Priority 2: share directory (installed)
             # Priority 3: source directory (dev)
             
             import ament_index_python.packages
             try:
                 pkg_share = ament_index_python.packages.get_package_share_directory('task_management')
                 rules_path_install = os.path.join(pkg_share, 'learned_rules.pl')
             except:
                 rules_path_install = "/non/existent/path"

             # Heuristic for source path: assumption based on file structure
             # .../task_management/task_management/allocation_strategy.py -> .../task_management/learned_rules.pl
             current_dir = os.path.dirname(os.path.abspath(__file__))
             pkg_root = os.path.dirname(current_dir)
             rules_path_source = os.path.join(pkg_root, 'learned_rules.pl')
             
             if os.path.exists(rules_path_install):
                 self.rules_path = rules_path_install
             elif os.path.exists(rules_path_source):
                 self.rules_path = rules_path_source
             else:
                 self.rules_path = None
                 self._log("WARN", f"[ILPAllocator] Could not find learned_rules.pl in install or source paths.")

             if self.rules_path:
                 self._log("INFO", f"[ILPAllocator] Loading rules from: {self.rules_path}")
                 self.janus.consult(self.rules_path)
                 self.use_ilp = True
                 self._log("INFO", f"[ILPAllocator] [SUCCESS] ILP Engine Ready.")
                 
        except Exception as e:
            self._log("WARN", f"[ILPAllocator] Failed to initialize ILP (Validation: {e}). Defaulting to Heuristic.")
            self.use_ilp = False

        # --- 2. Initialize Heuristic Fallback ---
        # We manually instantiate the Heuristic strategy to ensure we have a functional backup
        self.heuristic_fallback = HeuristicAllocator()
        
    def _log(self, level, msg):
        # Always print to stdout with flush=True to ensure visibility in all environments
        print(f"[{level}] {msg}", flush=True)
        # Also log to ROS logger if available
        if self.logger:
            if level == "INFO": self.logger.info(msg)
            elif level == "WARN": self.logger.warn(msg)
            elif level == "ERROR": self.logger.error(msg)

    def select_best_robot(self, robot_fleet_list, shelf_details, order, assigned_robots):
        if self.use_ilp:
            try:
                # --- ILP EXECUTION BLOCK ---
                best_robot_id, candidates = self._try_ilp_selection(robot_fleet_list, shelf_details, order, assigned_robots)
                if best_robot_id is not None:
                    # Success!
                    self.robot_workload[best_robot_id] += 1
                    return best_robot_id, f"ILP Rule Selected robot {best_robot_id}", "INFO", candidates
                else:
                    # Smart Fallback:
                    # If ILP returned no candidate, check if it's because NO robots are available (correct behavior)
                    # or because the rule failed to pick an available robot (failure).
                    
                    any_available = any(r.is_available and r.robot_id not in assigned_robots for r in robot_fleet_list)
                    
                    if not any_available:
                        self._log("INFO", f"[ILPAllocator] All robots busy. ILP correctly validated no assignment. Waiting...")
                        return None, None, None, []
                    else:
                        self._log("WARN", f"[ILPAllocator] ILP rule failed to pick available candidate. Fallback triggered.")
            except Exception as e:
                 import traceback
                 self._log("ERROR", f"[ILPAllocator] ILP Query crashed: {e}. Fallback triggered.\n{traceback.format_exc()}")

        # --- FALLBACK EXECUTION BLOCK ---
        return self.heuristic_fallback.select_best_robot(robot_fleet_list, shelf_details, order, assigned_robots)

    def _try_ilp_selection(self, robot_fleet_list, shelf_details, order, assigned_robots):
        """
        Private helper to execute the Prolog query. Returns (best_id, candidate_list).
        """
        # 1. Setup Context
        if not order.product_list: return None
        first_product_shelf_id = order.product_list[0].shelf_id
        target_shelf = next((s for s in shelf_details if s.shelf_id == first_product_shelf_id), None)
        if not target_shelf: return None
        shelf_location = target_shelf.shelf_location
        task_id = "current_task" 

        # 2. Gather Data & Create Facts
        facts_to_assert = []
        
        # Pre-calc stats for comparisons
        active_robots = []
        min_dist = float('inf')
        max_bat = -1.0
        
        # FIRST PASS: Calculate Global Stats across ALL robots
        # (This matches the new training logic where 'closest' is global)
        for robot in robot_fleet_list:
             dist = self._calculate_distance(robot.current_location, shelf_location)
             bat = float(robot.battery_level)
             
             if dist < min_dist: min_dist = dist
             if bat > max_bat: max_bat = bat

        # Calculates stats for idle robots
        min_dist_idle = float('inf')
        max_bat_idle = -1.0
        
        for robot in robot_fleet_list:
             is_busy = (robot.robot_id in assigned_robots) or (not robot.is_available)
             if not is_busy:
                 dist = self._calculate_distance(robot.current_location, shelf_location)
                 bat = float(robot.battery_level)
                 if dist < min_dist_idle: min_dist_idle = dist
                 if bat > max_bat_idle: max_bat_idle = bat

        # SECOND PASS: Build Facts
        for robot in robot_fleet_list:
             # We assume robots in assigned_robots are "busy" logic-wise
             # availability also comes from robot.is_available
             
             dist = self._calculate_distance(robot.current_location, shelf_location)
             bat = float(robot.battery_level)
             is_busy = (robot.robot_id in assigned_robots) or (not robot.is_available)
             is_idle = not is_busy

             active_robots.append({
                 "id": robot.robot_id,
                 "dist": dist,
                 "bat": bat,
                 "idle": is_idle
             })

             rid = f"robot_{robot.robot_id}"
            
             # Buckets
             bat_b = "high"
             if bat < 0.30: bat_b = "low"
             elif bat < 0.60: bat_b = "medium"
            
             dist_b = "far"
             if dist <= 3.0: dist_b = "near"
             elif dist <= 7.0: dist_b = "medium"
            
             wl_b = "zero"
             if self.robot_workload[robot.robot_id] > 0: wl_b = "low"
            
             facts_to_assert.append(f"battery_bucket({task_id}, {rid}, {bat_b})")
             facts_to_assert.append(f"distance_bucket({task_id}, {rid}, {dist_b})")
             facts_to_assert.append(f"workload_bucket({task_id}, {rid}, {wl_b})")
            
             if is_idle:
                facts_to_assert.append(f"idle({task_id}, {rid})")
             else:
                facts_to_assert.append(f"busy({task_id}, {rid})")
                
             # Comparative Facts (Global comparison)
             if dist <= min_dist + 0.01:
                 facts_to_assert.append(f"closest({task_id}, {rid})")
             if bat >= max_bat - 0.01:
                 facts_to_assert.append(f"most_charged({task_id}, {rid})")
                 
             if is_idle:
                 if dist <= min_dist_idle + 0.01:
                     facts_to_assert.append(f"closest_idle({task_id}, {rid})")
                 if bat >= max_bat_idle - 0.01:
                     facts_to_assert.append(f"most_charged_idle({task_id}, {rid})")

        # 3. Assert & Query
        self._log("INFO", f"--- ILP Step: Allocating Task (Global Context) ---")
        for f in facts_to_assert:
            self.janus.query_once(f"assertz({f})")
            # LOGGING: Show what the system "sees"
            if "closest" in f or "most_charged" in f or "idle" in f or "busy" in f:
                 self._log("INFO", f"  [FACT] {f}")
            
        q = f"assigned({task_id}, RobotArg)"
        best_id = None
        try:
            result = self.janus.query_once(q)
            self._log("INFO", f"  [QUERY] {q}  -> result: {result}")
            
            if result:
                robot_val = result.get('RobotArg')
                if robot_val and isinstance(robot_val, str) and robot_val.startswith("robot_"):
                    best_id = int(robot_val.split("_")[1])
                    self._log("INFO", f"  [DECISION] ILP selected Robot {best_id}")
                else:
                    self._log("INFO", f"  [DECISION] ILP returned result but no valid robot ID: {robot_val}")
            else:
                 self._log("INFO", f"  [DECISION] ILP found NO solution.")
        finally:
            # 4. Cleanup (Always retract)
            self.janus.query_once(f"retractall(battery_bucket({task_id}, _, _))")
            self.janus.query_once(f"retractall(distance_bucket({task_id}, _, _))")
            self.janus.query_once(f"retractall(workload_bucket({task_id}, _, _))")
            self.janus.query_once(f"retractall(idle({task_id}, _))")
            self.janus.query_once(f"retractall(busy({task_id}, _))")
            self.janus.query_once(f"retractall(closest({task_id}, _))")
            self.janus.query_once(f"retractall(most_charged({task_id}, _))")
            self.janus.query_once(f"retractall(closest_idle({task_id}, _))")
            self.janus.query_once(f"retractall(most_charged_idle({task_id}, _))")
            
        # Reformat candidates for logger
        # Keys needed: candidate_robot, idle, workload, battery, distance, battery_bucket, distance_bucket
        final_candidates = []
        for r in active_robots:
            # Buckets
            bat_b = "high"
            if r["bat"] < 0.30: bat_b = "low"
            elif r["bat"] < 0.60: bat_b = "medium"
            
            dist_b = "far"
            if r["dist"] <= 3.0: dist_b = "near"
            elif r["dist"] <= 7.0: dist_b = "medium"
            
            wl_b = "zero"
            if self.robot_workload[r['id']] > 0: wl_b = "low"
            
            # Add heuristic score for comparison/logging
            score = (1 / (r["dist"] + 1)) * r["bat"]
            
            final_candidates.append({
                "robot": r["id"],
                "battery": r["bat"],
                "distance": r["dist"],
                "workload": self.robot_workload[r['id']],
                "idle": r["idle"],
                "battery_bucket": bat_b,
                "distance_bucket": dist_b,
                "workload_bucket": wl_b,
                "score": score
            })
            
        return best_id, final_candidates

    def on_task_completion(self, robot_id):
        if self.robot_workload[robot_id] > 0:
            self.robot_workload[robot_id] -= 1

    def _calculate_distance(self, location1: Pose, location2: Pose):
        return math.sqrt(
            (location2.position.x - location1.position.x) ** 2 + 
            (location2.position.y - location1.position.y) ** 2
        )
