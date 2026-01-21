prediction(chosen, 1).
type(chosen, (robot,)).
type(dist_bucket, (robot, dist_label)).
type(batt_bucket, (robot, batt_label)).
type(workload, (robot, int)).
type(is_idle, (robot, bool)).

body_pred(dist_bucket, 2).
body_pred(batt_bucket, 2).
body_pred(workload, 2).
body_pred(is_idle, 2).

constant(dist_label, medium).
constant(dist_label, near).
constant(batt_label, medium).
constant(batt_label, low).
constant(batt_label, high).
constant(bool, true).
constant(bool, false).
