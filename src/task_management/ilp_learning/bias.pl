% Head predicate: assigned(Task, Robot)
head_pred(assigned, 2).

% Body predicates
body_pred(battery_bucket, 3).
body_pred(distance_bucket, 3).
body_pred(idle, 2).
body_pred(busy, 2).
body_pred(closest, 2).
body_pred(workload_bucket, 3).
body_pred(most_charged, 2).

% Types
type(assigned, (task, robot)).
type(battery_bucket, (task, robot, bat_level)).
type(distance_bucket, (task, robot, dist_level)).
type(idle, (task, robot)).
type(busy, (task, robot)).
type(closest, (task, robot)).
type(workload_bucket, (task, robot, wl_level)).
type(most_charged, (task, robot)).

% Direction
direction(assigned, (in, in)).
direction(battery_bucket, (in, in, out)).
direction(distance_bucket, (in, in, out)).
direction(idle, (in, in)).
direction(busy, (in, in)).
direction(closest, (in, in)).
direction(workload_bucket, (in, in, out)).
direction(most_charged, (in, in)).

% Constraints
max_vars(4).
max_body(5).
