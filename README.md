# ILP-Based Task Allocation System

This project implements a multi-robot task distribution system designed for warehouse automation. It features a unique **Inductive Logic Programming (ILP)** component that learns task allocation strategies from historical data, allowing the system to adapt its behavior (e.g., favoring closest robots vs. robots with high battery) based on successful past assignments.

## Features

*   **Multi-Robot Simulation**: Simulates a warehouse environment with multiple robots using ROS2 and Gazebo.
*   **ILP-Based Allocation**: Uses **Popper** (an ILP system) to learn logic rules from logs.
*   **Robust Fallback**: Automatically switches to a heuristic strategy if ILP rules are unavailable or fail.
*   **Web Dashboard**: A real-time interface to submit orders and view robot status.
*   **Automated Pipeline**: Tools to process logs, generate Prolog facts, and train new rules.

---

## Installation

### Prerequisites
*   Ubuntu 22.04 (Jammy)
*   ROS2 Humble
*   Python 3.10+
*   [Popper](https://github.com/logic-and-learning-lab/Popper) (Validation required)
*   SWI-Prolog (`swi-prolog`)

### 1. Create Workspace
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/ujjwalpatil20/ilp-based-task-allocation.git
```

### 2. Install Dependencies
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
pip install flask popper-ilp janus-swi
```

### 3. Build
```bash
colcon build --symlink-install
source install/setup.bash
```

---

## Running the System

To run the simulation and the task management system, you will need two terminals.

### Terminal 1: Simulation Environment
Sets up the Gazebo world, robot models, and navigation stack.
```bash
source install/setup.bash
export NUM_OF_ROBOTS=2  # Set the desired number of robots
ros2 launch simulation_env sim_with_nav.launch.py
```

### Terminal 2: Warehouse Manager
Launches the Task Manager, Fleet Manager, and Web Server.
```bash
source install/setup.bash
export NUM_OF_ROBOTS=2
ros2 launch simulation_env warehouse_manager.launch.py
```

**Access the Web Dashboard** at `http://localhost:5000` to submit orders.

---

## ILP Learning Pipeline

The unique feature of this project is its ability to learn allocation rules. Here is how the training works.

### 1. Data Collection (Logging)
The system automatically logs every allocation attempt to `src/task_management/ilp_learning/ilp_logs/`.
The log captures:
*   Robot distances to target.
*   Battery levels.
*   Workload.
*   Whether the assignment was successful.

### 2. Learning Process
You can retrain the system using the provided scripts. This process converts logs into Logic Programming (Prolog) facts and uses the Popper ILP engine to find a rule that explains the data.

**Run the Full Pipeline:**
```bash
cd src/task_management/ilp_learning
./test_ilp_pipeline.sh
```

**What this script does:**
1.  **`generate_ilp_data.py`**: Reads the latest CSV log files and generates two Prolog files:
    *   `bk.pl` (Background Knowledge): Static facts and helper predicates.
    *   `exs.pl` (Examples): Positive and negative examples derived from the logs (e.g., `pos(assigned(task1, robot_1))`).
2.  **`run_ilp.py`**: Invokes the **Popper** library using the generated files and `bias.pl` (Language Bias).
3.  **Result**: If a rule is found (e.g., `assigned(T, R) :- ...`), it is saved to `src/task_management/learned_rules.pl`.

### 3. Using Learned Rules
The `ILPAllocator` class in `allocation_strategy.py` automatically checks for `src/task_management/learned_rules.pl`.
*   If found, it loads the rule using `janus_swi` (Python-Prolog bridge) and uses it to decide which robot gets the task.
*   If not found (or if the rule fails), it gracefully falls back to the hardcoded `HeuristicAllocator`.

---

## Directory Structure

*   `src/simulation_env`: Launch files, world maps, and navigation configs.
*   `src/task_management`: Core logic nodes.
    *   `allocation_strategy.py`: The brain that decides which robot picks an order.
    *   `ilp_learning/`: The "Machine Learning" studio.
        *   `bk.pl`, `bias.pl`: Prolog templates for learning.
        *   `run_ilp.py`: The trainer script.
*   `src/fleet_manager`: Handles low-level robot control and status updates.

---

## License
MIT License
