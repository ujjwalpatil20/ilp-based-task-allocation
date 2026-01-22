import csv
import os
import glob
import sys
from collections import defaultdict

def get_latest_csv(log_dir):
    list_of_files = glob.glob(os.path.join(log_dir, '*.csv'))
    if not list_of_files:
        return None
    return max(list_of_files, key=os.path.getctime)

def generate_popper_files(csv_file, output_dir):
    bk_file = os.path.join(output_dir, 'bk.pl')
    exs_file = os.path.join(output_dir, 'exs.pl')
    
    facts = set()
    pos_exs = set()
    neg_exs = set()
    
    # Group by task to compare candidates
    tasks = defaultdict(list)
    
    print(f"Reading {csv_file}...")
    
    # Group by (task_id, timestamp) to identify unique allocation events
    events = defaultdict(list)
    
    print(f"Reading {csv_file}...")
    
    with open(csv_file, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
             # Use timestamp to differentiate retries or reused IDs
             key = (row['task_id'], row['timestamp'])
             events[key].append(row)

    event_counter = 0
    for (original_task_id, ts), rows in events.items():
        event_counter += 1
        # Create a unique ID for this specific decision event
        # e.g. order_1_evt001
        clean_task_id = original_task_id.lower().replace('-', '_')
        task_id = f"{clean_task_id}_evt{event_counter}"
        
        # Find best stats AMONG ALL ROBOTS
        # idle_rows = [r for r in rows if r['idle'].lower() == 'true']
        
        # Determine global min/max for this task event
        min_dist = float('inf')
        max_bat = -1.0
        
        if rows:
            min_dist = min(float(r['distance']) for r in rows)
            max_bat = max(float(r['battery']) for r in rows)
        
        for row in rows:
            robot_id = f"robot_{row['candidate_robot']}"
            
            # Values
            bat_bucket = row['battery_bucket'].lower()
            dist_bucket = row['distance_bucket'].lower()
            is_idle = row['idle'].lower() == 'true'
            label = int(row['label'])
            dist = float(row['distance'])
            bat = float(row['battery'])
            workload = int(row.get('workload', 0))
            
            # Bucketing Workload
            wl_bucket = 'zero'
            if workload > 2:
                wl_bucket = 'high'
            elif workload > 0:
                wl_bucket = 'low'
            
            # BK Facts
            # battery_bucket(Task, Robot, Level)
            facts.add(f"battery_bucket({task_id}, {robot_id}, {bat_bucket}).")
            
            # distance_bucket(Task, Robot, Level)
            facts.add(f"distance_bucket({task_id}, {robot_id}, {dist_bucket}).")
            
            # workload_bucket(Task, Robot, Level)
            facts.add(f"workload_bucket({task_id}, {robot_id}, {wl_bucket}).")
            
            # idle(Task, Robot)
            if is_idle:
                facts.add(f"idle({task_id}, {robot_id}).")
            else:
                facts.add(f"busy({task_id}, {robot_id}).")
            
            # Comparative Facts (Global comparison)
            if dist <= min_dist + 0.01:
                facts.add(f"closest({task_id}, {robot_id}).")
    
            if bat >= max_bat - 0.01:
                facts.add(f"most_charged({task_id}, {robot_id}).")

            # Examples
            # assigned(Task, Robot)
            ex = f"assigned({task_id}, {robot_id})"
            if label == 1:
                pos_exs.add(f"pos({ex}).")
            else:
                neg_exs.add(f"neg({ex}).")
                
    # Write BK
    with open(bk_file, 'w') as f:
        f.write(":- dynamic battery_bucket/3, distance_bucket/3, workload_bucket/3, idle/2, busy/2, closest/2, most_charged/2.\n")
        f.write("\n".join(sorted(facts)))
        print(f"Wrote {len(facts)} facts to {bk_file}")

    # Write Exs
    with open(exs_file, 'w') as f:
        f.write("\n".join(sorted(pos_exs)))
        f.write("\n")
        f.write("\n".join(sorted(neg_exs)))
        print(f"Wrote {len(pos_exs)} pos and {len(neg_exs)} neg examples to {exs_file}")
        
    if len(pos_exs) == 0:
        print("\n[WARNING] No positive examples were generated!")
        print("Possible causes:")
        print("1. You are analyzing an old log file (Check the timestamp printed above).")
        print("2. The 'assigned_robot' in logs does not match any 'candidate_robot'.")
        print("Recommendation: Run 'ros2 run task_management ilp_dataset_logger', place new orders, and try again.\n")

if __name__ == "__main__":
    # Default paths
    # workspace_root = "/home/ibrahim/ros2_ws/src/ilp-based-task-allocation"
    # log_dir = os.path.join(workspace_root, "ilp_logs")
    
    # Use script location as anchor
    output_dir = os.path.dirname(os.path.abspath(__file__))
    log_dir = os.path.join(output_dir, "ilp_logs")
    
    if len(sys.argv) > 1:
        log_dir = sys.argv[1]
        
    latest_csv = get_latest_csv(log_dir)
    if not latest_csv:
        print("No CSV files found in", log_dir)
        sys.exit(1)
        
    generate_popper_files(latest_csv, output_dir)
