
import unittest
import sys
import os
from collections import namedtuple, defaultdict

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

class TestNewILP(unittest.TestCase):
    def setUp(self):
        # The allocator automatically looks for learned_rules.pl in local dir
        # We assume the test is running in an environment where it can find it, 
        # or we rely on the heuristic in allocation_strategy.py
        self.allocator = ILPAllocator(logger=MockLogger())
        
        # Ensure ILP loaded
        if not self.allocator.use_ilp:
            print("WARNING: ILP not loaded in test! Check path to learned_rules.pl")
            
    def test_strict_rule_behavior(self):
        """
        Rule: assigned(A,B) :- most_charged(A,B), idle(A,B), closest(A,B).
        This requires a robot to be BOTH the closest AND the most charged.
        """
        if not self.allocator.use_ilp:
            self.skipTest("ILP engine not loaded")

        shelf = ShelfStatus(shelf_id='s1', shelf_location=Pose(position=Position(x=0.0, y=0.0)))
        order = Order(product_list=[Product(shelf_id='s1')])
        
        # Scenario 1: No perfect candidate
        # R1: Average dist(5), High bat(0.9) -> Most Charged
        # R2: Small dist(1), Low bat(0.2)   -> Closest
        # Result: ILP should return None (fallback) because no robot is BOTH.
        r1 = RobotStatus(robot_id=1, is_available=True, current_location=Pose(position=Position(x=5.0, y=0.0)), battery_level=0.9)
        r2 = RobotStatus(robot_id=2, is_available=True, current_location=Pose(position=Position(x=1.0, y=0.0)), battery_level=0.2)
        
        # Run Scenario 1
        best_id, msg, _, _ = self.allocator.select_best_robot([r1, r2], [shelf], order, set())
        print(f"\nScenario 1 Result: {best_id}, Msg: {msg}")
        
        # Expectation: ILP fails (strict rule), so it falls back to Heuristic.
        # Heuristic picks Robot 1 (higher battery).
        # So we expect best_id=1, but msg should NOT indicate ILP.
        self.assertEqual(best_id, 1, "Heuristic should still pick a robot")
        self.assertNotRegex(str(msg), r"ILP Rule Selected", "Should NOT be an ILP decision")
        
        # Scenario 2: Perfect candidate
        # R3: Dist(1), Bat(0.9) -> Closest AND Most Charged
        r3 = RobotStatus(robot_id=3, is_available=True, current_location=Pose(position=Position(x=1.0, y=0.0)), battery_level=0.9)
        
        # R4: Dist(10), Bat(0.1) -> Junk
        r4 = RobotStatus(robot_id=4, is_available=True, current_location=Pose(position=Position(x=10.0, y=0.0)), battery_level=0.1)

        best_id_2, msg_2, _, _ = self.allocator.select_best_robot([r3, r4], [shelf], order, set())
        print(f"Scenario 2 Result: {best_id_2}, Msg: {msg_2}")
        
        self.assertEqual(best_id_2, 3, "Should select robot 3")
        self.assertRegex(str(msg_2), r"ILP Rule Selected", "Should BE an ILP decision")

if __name__ == '__main__':
    unittest.main()
