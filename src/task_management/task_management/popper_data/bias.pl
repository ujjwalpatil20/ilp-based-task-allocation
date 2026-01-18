prediction(chosen, 1).
type(chosen, (robot,)).
type(dist_bucket, (robot, dist_label)).
type(batt_bucket, (robot, batt_label)).
type(workload, (robot, int)).

body_pred(dist_bucket, 2).
body_pred(batt_bucket, 2).
body_pred(workload, 2).

constant(dist_label, very_near).
constant(dist_label, near).
constant(dist_label, far).
constant(batt_label, low).
constant(batt_label, medium).
constant(batt_label, high).