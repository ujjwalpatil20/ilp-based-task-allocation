
import unittest
import sys
import os
from collections import namedtuple

# Add package root to path so we can import modules
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../')))

from task_management.allocation_strategy import ILPAllocator

# Mocks
RobotStatus = namedtuple('RobotStatus', ['robot_id', 'is_available', 'current_location', 'battery_level'])
ShelfStatus = namedtuple('ShelfStatus', ['shelf_id', 'shelf_location'])
Order = namedtuple('Order', ['product_list'])
Product = namedtuple('Product', ['shelf_id'])
Pose = namedtuple('Pose', ['position'])
Position = namedtuple('Position', ['x', 'y'])

class TestILPRuntime(unittest.TestCase):
    def setUp(self):
        # We know where the rules are: src/task_management/learned_rules.pl
        # The test is in src/task_management/test/test_ilp_runtime.py
        
        test_dir = os.path.dirname(os.path.abspath(__file__))
        rules_path = os.path.join(test_dir, '../learned_rules.pl')
        
        if not os.path.exists(rules_path):
             # Try assuming we are in src/task_management root
             rules_path = os.path.abspath("learned_rules.pl")
             
        self.allocator = ILPAllocator(rules_path=rules_path)
        
    def test_allocator_loads_janus(self):
        self.assertTrue(self.allocator.janus_loaded, "Janus should be loaded")
        
    def test_closest_robot_selection(self):
        # Setup: Robot 1 is closer, Robot 2 is farther. Both idle.
        # Rule expected: assigned(V0,V1):- closest(V0,V1).
        
        # Shelf at (0,0)
        shelf = ShelfStatus(shelf_id='s1', shelf_location=Pose(position=Position(x=0.0, y=0.0)))
        
        # Robot 1 at (1,0) -> dist 1
        r1 = RobotStatus(robot_id=1, is_available=True, current_location=Pose(position=Position(x=1.0, y=0.0)), battery_level=0.8)
        
        # Robot 2 at (10,0) -> dist 10
        r2 = RobotStatus(robot_id=2, is_available=True, current_location=Pose(position=Position(x=10.0, y=0.0)), battery_level=0.8)
        
        order = Order(product_list=[Product(shelf_id='s1')])
        
        # Run
        best_robot, msg, level, candidates = self.allocator.select_best_robot(
            robot_fleet_list=[r1, r2],
            shelf_details=[shelf],
            order=order,
            assigned_robots=set()
        )
        
        print(f"Debug Candidates: {candidates}")
        print(f"Result: {best_robot}, {msg}")
        
        self.assertEqual(best_robot, 1, "Should select robot 1 because it is closest")

if __name__ == '__main__':
    unittest.main()
