# ILP Based Task Distribution System

This project implements a multi-robot task distribution system designed for warehouse automation, enhanced with Induction Logic Programming (ILP) concepts for intelligent task allocation.

### For details about the core architecture and node documentation, please refer to the original repository:
[Original Repository README](https://github.com/HBRS-SDP/ws24-multi-robot-task-distribution/blob/8c7dc51ea050e1eed7ebb6f6c8ebd692007cface/README.md)

---

## ILP Process & Task Allocation

The core of this fork is the integration of an ILP-inspired hybrid allocation strategy. The system evaluates the fleet state in real-time and applies a 3-step priority logic to ensure both efficiency and fleet health.

### 3-Step Hybrid Logic:
1.  **Overwork Prevention**: Robots with a current workload higher than a specific threshold are deprioritized to prevent mechanical strain and ensure load balancing.
2.  **Survival Mode (Battery Priority)**: If a robot's battery level drops below 40%, it enters a "Survival Mode" where battery level becomes the primary metric for assignment, ensuring robots don't die in the field.
3.  **Efficiency Mode (Proximity Priority)**: Under normal conditions, the system defaults to Efficiency Mode, selecting the closest available robot to the target shelf to minimize travel time.

### Data Logging for ILP Training:
The system includes an `ILPDatasetLogger` that captures all allocation events. Each log entry includes:
- Robot proximity to the task.
- Battery levels.
- Current workloads.
- The final assignment decision.
This dataset can be used to further train and refine the logic using ILP solvers like Popper.

---

## Installation and Project Setup

### Prerequisites
- ROS2 Humble
- Git
- Python 3

### 1. Create a ROS Workspace
```bash
mkdir -p ~/ros_ws/src
cd ~/ros_ws/
colcon build
source install/setup.bash
```

### 2. Clone the Package
```bash
cd ~/ros_ws/src
git clone https://github.com/ujjwalpatil20/ilp-based-task-allocation.git
```

### 3. Install Dependencies
```bash
cd ~/ros_ws
rosdep install --from-paths src --ignore-src -r -y
pip install flask
```

### 4. Build the Package
```bash
colcon build --symlink-install
```

---

## Launching the System

### 1. Source the Workspace
```bash
source ~/ros_ws/install/setup.bash
```

### 2. Set the Number of Robots
```bash
export NUM_OF_ROBOTS=2  # Default is 2
```

### 3. Launch the Simulation Environment
```bash
ros2 launch simulation_env sim_with_nav.launch.py
```

### 4. Launch the Warehouse Manager
This launches the Task Manager, Fleet Manager, Shared Memory, and the Web GUI.
```bash
ros2 launch simulation_env warehouse_manager.launch.py
```

### 5. Access the Web Client
The GUI will auto-launch or be accessible at: `http://localhost:5000`

---

## Workflow Summary
1. **Order Placement**: Users submit orders via the Web GUI.
2. **Task Evaluation**: The Task manager uses the `ILPAllocator` to score available robots based on Workload, Battery, and Distance.
3. **Task Assignment**: The highest-scoring robot is assigned the task via the Fleet Manager.
4. **Execution**: The robot navigates through waypoints (Shelves -> Drop-off).
5. **Completion**: Once "Idle" status is confirmed, the order is marked complete and the robot's workload is decremented.

---

## License
This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.
