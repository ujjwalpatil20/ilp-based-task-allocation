% Learned from /home/ibrahim/ros2_ws/src/ilp-based-task-allocation/src/task_management/ilp_learning
:- dynamic assigned/2.
assigned(V0,V1):- closest(V0,V1).
