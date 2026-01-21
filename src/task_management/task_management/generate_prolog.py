# import pandas as pd
# import os

# # Where your data is currently saved
# csv_path = '/home/alisha/ros2_ws/src/ilp-based-task-allocation/src/task_management/task_management/warehouse_training_data.csv'

# # The folder we will create for your friend
# output_dir = 'popper_data'
# os.makedirs(output_dir, exist_ok=True)

# # Load the CSV
# df = pd.read_csv(csv_path)

# # Group rows by timestamp so Popper knows which robots were in the same 'scene'
# df['task_id'] = pd.factorize(df['timestamp'])[0]

# with open(f'{output_dir}/exs.pl', 'w') as f_exs, open(f'{output_dir}/bk.pl', 'w') as f_bk:
#     for _, row in df.iterrows():
#         # Unique ID for the robot instance (e.g., r1_t0)
#         obj_id = f"r{int(row['robot_id'])}_t{row['task_id']}"
        
#         # Write the Facts (Background Knowledge)
#         f_bk.write(f"dist_bucket({obj_id}, {row['distance_bucket']}).\n")
#         f_bk.write(f"batt_bucket({obj_id}, {row['battery_bucket']}).\n")
#         f_bk.write(f"workload({obj_id}, {int(row['workload'])}).\n")
        
#         # Write the Examples (Positive/Negative)
#         if row['is_chosen'] == 1:
#             f_exs.write(f"pos(chosen({obj_id})).\n")
#         else:
#             f_exs.write(f"neg(chosen({obj_id})).\n")

# print(f"Success! Files created in: {os.path.abspath(output_dir)}")
import pandas as pd
import os

# 1. Update this to your teammate's file name
csv_path = '/home/alisha/ros2_ws/src/ilp-based-task-allocation/src/task_management/task_management/ilp_dataset_2026-01-21_23-37-59.csv'
output_dir = 'popper_data'
os.makedirs(output_dir, exist_ok=True)

# Load the new CSV
df = pd.read_csv(csv_path)

# Popper logic: Create examples (exs.pl) and background knowledge (bk.pl)
with open(f'{output_dir}/exs.pl', 'w') as f_exs, open(f'{output_dir}/bk.pl', 'w') as f_bk:
    for _, row in df.iterrows():
        # Clean task_id for Prolog (it doesn't like hyphens like 'order-1')
        task_id = str(row['task_id']).replace('-', '_')
        # Unique ID for this specific candidate: e.g., rob_order_1_1
        rob_id = f"rob_{task_id}_{row['candidate_robot']}"
        
        # --- 1. Background Knowledge (bk.pl) ---
        f_bk.write(f"dist_bucket({rob_id}, {row['distance_bucket']}).\n")
        f_bk.write(f"batt_bucket({rob_id}, {row['battery_bucket']}).\n")
        f_bk.write(f"workload({rob_id}, {int(row['workload'])}).\n")
        # Adding 'idle' as a fact since your teammate included it
        idle_val = 'true' if row['idle'] else 'false'
        f_bk.write(f"is_idle({rob_id}, {idle_val}).\n")
        
        # --- 2. Examples (exs.pl) ---
        # Using 'label' column: 1 is positive, 0 is negative
        if row['label'] == 1:
            f_exs.write(f"pos(chosen({rob_id})).\n")
        else:
            f_exs.write(f"neg(chosen({rob_id})).\n")

# --- 3. Generate bias.pl (The AI Instructions) ---
# We automatically grab the labels (near, medium, high, etc.) from the data
dist_labels = df['distance_bucket'].unique()
batt_labels = df['battery_bucket'].unique()

bias_content = f"""prediction(chosen, 1).
type(chosen, (robot,)).
type(dist_bucket, (robot, dist_label)).
type(batt_bucket, (robot, batt_label)).
type(workload, (robot, int)).
type(is_idle, (robot, bool)).

body_pred(dist_bucket, 2).
body_pred(batt_bucket, 2).
body_pred(workload, 2).
body_pred(is_idle, 2).

/* Constants detected in your teammate's data */
"""
for d in dist_labels: bias_content += f"constant(dist_label, {d}).\n"
for b in batt_labels: bias_content += f"constant(batt_label, {b}).\n"
bias_content += "constant(bool, true).\nconstant(bool, false).\n"

with open(f'{output_dir}/bias.pl', 'w') as f_bias:
    f_bias.write(bias_content)

print(f"Success! Integrated teammate's heuristic data. Files saved in: {os.path.abspath(output_dir)}")