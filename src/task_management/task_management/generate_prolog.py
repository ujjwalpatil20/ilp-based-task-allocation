import pandas as pd
import os

# Where your data is currently saved
csv_path = '/home/alisha/ros2_ws/src/ilp-based-task-allocation/src/task_management/task_management/warehouse_training_data.csv'

# The folder we will create for your friend
output_dir = 'popper_data'
os.makedirs(output_dir, exist_ok=True)

# Load the CSV
df = pd.read_csv(csv_path)

# Group rows by timestamp so Popper knows which robots were in the same 'scene'
df['task_id'] = pd.factorize(df['timestamp'])[0]

with open(f'{output_dir}/exs.pl', 'w') as f_exs, open(f'{output_dir}/bk.pl', 'w') as f_bk:
    for _, row in df.iterrows():
        # Unique ID for the robot instance (e.g., r1_t0)
        obj_id = f"r{int(row['robot_id'])}_t{row['task_id']}"
        
        # Write the Facts (Background Knowledge)
        f_bk.write(f"dist_bucket({obj_id}, {row['distance_bucket']}).\n")
        f_bk.write(f"batt_bucket({obj_id}, {row['battery_bucket']}).\n")
        f_bk.write(f"workload({obj_id}, {int(row['workload'])}).\n")
        
        # Write the Examples (Positive/Negative)
        if row['is_chosen'] == 1:
            f_exs.write(f"pos(chosen({obj_id})).\n")
        else:
            f_exs.write(f"neg(chosen({obj_id})).\n")

print(f"Success! Files created in: {os.path.abspath(output_dir)}")