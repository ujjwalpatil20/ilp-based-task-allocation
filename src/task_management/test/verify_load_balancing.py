
import unittest
import sys
import os
from collections import namedtuple

# Add package root to path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../')))

from task_management.allocation_strategy import ILPAllocator

# Mocks
RobotStatus = namedtuple('RobotStatus', ['robot_id', 'is_available', 'current_location', 'battery_level'])
ShelfStatus = namedtuple('ShelfStatus', ['shelf_id', 'shelf_location'])
Order = namedtuple('Order', ['product_list'])
Product = namedtuple('Product', ['shelf_id'])
Pose = namedtuple('Pose', ['position'])
Position = namedtuple('Position', ['x', 'y'])

class MockLogger:
    def info(self, msg): print(f"[TEST INFO] {msg}")
    def warn(self, msg): print(f"[TEST WARN] {msg}")
    def error(self, msg): print(f"[TEST ERROR] {msg}")

class TestLoadBalancing(unittest.TestCase):
    def setUp(self):
        self.allocator = ILPAllocator(logger=MockLogger())
        if not self.allocator.use_ilp:
            print("WARNING: ILP not loaded! Test checks nothing.")
            
    def test_idle_priority(self):
        """
        Verify that an IDLE robot is chosen over a BUSY robot, 
        even if the busy robot is 'better' (closer/more charged).
        """
        if not self.allocator.use_ilp:
            self.skipTest("ILP engine not loaded")

        shelf = ShelfStatus(shelf_id='s1', shelf_location=Pose(position=Position(x=0.0, y=0.0)))
        order = Order(product_list=[Product(shelf_id='s1')])
        
        # Robot 1: Closest (1.0), Max Battery (1.0), BUT BUSY
        # Robot 2: Further (5.0), Good Battery (0.9), IDLE
        
        # Note: assigned_robots set makes them 'busy' in eyes of ILP
        assigned_robots = {1} 
        
        r1 = RobotStatus(robot_id=1, is_available=True, current_location=Pose(position=Position(x=1.0, y=0.0)), battery_level=1.0)
        r2 = RobotStatus(robot_id=2, is_available=True, current_location=Pose(position=Position(x=5.0, y=0.0)), battery_level=0.9)
        
        # Rules:
        # 1. idle, most_charged, closest
        # 2. idle, closest
        # 3. most_charged, closest
        # 4. closest
        
        # R1 matches #3 (busy, most_charged, closest)
        # R2 matches #2 (idle, closest among idle?) 
        # Actually R2 is NOT 'closest' global. 'closest(A,B)' is global.
        # Wait, if 'closest' predicate relies on global min, R2 is NOT closest.
        
        # Let's check how facts are generated in allocation_strategy.py:
        # facts_to_assert.append(f"closest({task_id}, {rid})") if dist <= min_dist
        
        # So 'closest' is STRICTLY the global minimum. 
        # R1 is global closest. R2 is NOT global closest.
        
        # If the rule is `assigned :- idle, closest`, it requires the robot to be IDLE AND GLOBAL CLOSEST.
        # If R1 is closest but busy, NO ONE is "idle and closest".
        
        # We need to verify if the rule structure supports "Best Available".
        # If we stick to strict "closest" predicate, we might still fail to pick R2.
        
        # Let's run the test to see what happens with current logic.
        
        best_id, msg, _, candidate_list = self.allocator.select_best_robot([r1, r2], [shelf], order, assigned_robots)
        print(f"\nLoad Balancing Result: {best_id}, Msg: {msg}")
        
        # If R1 is selected, we failed load balancing.
        # If R2 is selected, we succeeded.
        # If None, we failed.
        
        # Based on my analysis, if R2 is not 'closest', it won't be picked by 'idle, closest'.
        # We might need to handle this.
        
        self.assertEqual(best_id, 2, "Should pick Robot 2 (Idle) over Robot 1 (Busy)")

if __name__ == '__main__':
    unittest.main()
