:- dynamic battery_bucket/3, distance_bucket/3, workload_bucket/3, idle/2, busy/2, closest/2, most_charged/2.
battery_bucket(order_10_evt12, robot_1, high).
battery_bucket(order_10_evt12, robot_2, high).
battery_bucket(order_11_evt13, robot_1, medium).
battery_bucket(order_11_evt13, robot_2, medium).
battery_bucket(order_12_evt14, robot_1, medium).
battery_bucket(order_12_evt14, robot_2, medium).
battery_bucket(order_13_evt15, robot_1, medium).
battery_bucket(order_13_evt15, robot_2, medium).
battery_bucket(order_14_evt16, robot_1, high).
battery_bucket(order_14_evt16, robot_2, high).
battery_bucket(order_15_evt17, robot_1, high).
battery_bucket(order_15_evt17, robot_2, high).
battery_bucket(order_16_evt18, robot_1, medium).
battery_bucket(order_16_evt18, robot_2, medium).
battery_bucket(order_17_evt19, robot_1, medium).
battery_bucket(order_17_evt19, robot_2, medium).
battery_bucket(order_18_evt20, robot_1, low).
battery_bucket(order_18_evt20, robot_2, low).
battery_bucket(order_19_evt21, robot_1, low).
battery_bucket(order_19_evt21, robot_2, low).
battery_bucket(order_1_evt1, robot_1, high).
battery_bucket(order_1_evt1, robot_2, high).
battery_bucket(order_1_evt3, robot_1, high).
battery_bucket(order_1_evt3, robot_2, high).
battery_bucket(order_20_evt22, robot_1, high).
battery_bucket(order_20_evt22, robot_2, high).
battery_bucket(order_21_evt23, robot_1, high).
battery_bucket(order_21_evt23, robot_2, high).
battery_bucket(order_22_evt24, robot_1, high).
battery_bucket(order_22_evt24, robot_2, high).
battery_bucket(order_23_evt25, robot_1, medium).
battery_bucket(order_23_evt25, robot_2, medium).
battery_bucket(order_2_evt2, robot_1, high).
battery_bucket(order_2_evt2, robot_2, high).
battery_bucket(order_2_evt4, robot_1, high).
battery_bucket(order_2_evt4, robot_2, high).
battery_bucket(order_3_evt5, robot_1, medium).
battery_bucket(order_3_evt5, robot_2, medium).
battery_bucket(order_4_evt6, robot_1, medium).
battery_bucket(order_4_evt6, robot_2, medium).
battery_bucket(order_5_evt7, robot_1, low).
battery_bucket(order_5_evt7, robot_2, low).
battery_bucket(order_6_evt8, robot_1, medium).
battery_bucket(order_6_evt8, robot_2, medium).
battery_bucket(order_7_evt9, robot_1, high).
battery_bucket(order_7_evt9, robot_2, high).
battery_bucket(order_8_evt10, robot_1, high).
battery_bucket(order_8_evt10, robot_2, high).
battery_bucket(order_9_evt11, robot_1, high).
battery_bucket(order_9_evt11, robot_2, high).
busy(order_10_evt12, robot_2).
busy(order_11_evt13, robot_1).
busy(order_12_evt14, robot_2).
busy(order_13_evt15, robot_2).
busy(order_15_evt17, robot_2).
busy(order_16_evt18, robot_2).
busy(order_17_evt19, robot_1).
busy(order_18_evt20, robot_2).
busy(order_19_evt21, robot_1).
busy(order_20_evt22, robot_1).
busy(order_21_evt23, robot_1).
busy(order_22_evt24, robot_2).
busy(order_23_evt25, robot_1).
busy(order_2_evt2, robot_2).
busy(order_2_evt4, robot_1).
busy(order_3_evt5, robot_2).
busy(order_4_evt6, robot_1).
busy(order_5_evt7, robot_2).
busy(order_6_evt8, robot_1).
busy(order_7_evt9, robot_2).
busy(order_8_evt10, robot_1).
busy(order_9_evt11, robot_1).
closest(order_10_evt12, robot_1).
closest(order_11_evt13, robot_2).
closest(order_12_evt14, robot_1).
closest(order_13_evt15, robot_1).
closest(order_14_evt16, robot_2).
closest(order_15_evt17, robot_1).
closest(order_16_evt18, robot_1).
closest(order_17_evt19, robot_2).
closest(order_18_evt20, robot_1).
closest(order_19_evt21, robot_2).
closest(order_1_evt1, robot_2).
closest(order_1_evt3, robot_1).
closest(order_20_evt22, robot_2).
closest(order_21_evt23, robot_2).
closest(order_22_evt24, robot_1).
closest(order_23_evt25, robot_2).
closest(order_2_evt2, robot_1).
closest(order_2_evt4, robot_2).
closest(order_3_evt5, robot_1).
closest(order_4_evt6, robot_2).
closest(order_5_evt7, robot_1).
closest(order_6_evt8, robot_2).
closest(order_7_evt9, robot_1).
closest(order_8_evt10, robot_2).
closest(order_9_evt11, robot_2).
distance_bucket(order_10_evt12, robot_1, medium).
distance_bucket(order_10_evt12, robot_2, near).
distance_bucket(order_11_evt13, robot_1, near).
distance_bucket(order_11_evt13, robot_2, near).
distance_bucket(order_12_evt14, robot_1, near).
distance_bucket(order_12_evt14, robot_2, near).
distance_bucket(order_13_evt15, robot_1, near).
distance_bucket(order_13_evt15, robot_2, near).
distance_bucket(order_14_evt16, robot_1, medium).
distance_bucket(order_14_evt16, robot_2, medium).
distance_bucket(order_15_evt17, robot_1, near).
distance_bucket(order_15_evt17, robot_2, near).
distance_bucket(order_16_evt18, robot_1, medium).
distance_bucket(order_16_evt18, robot_2, medium).
distance_bucket(order_17_evt19, robot_1, near).
distance_bucket(order_17_evt19, robot_2, medium).
distance_bucket(order_18_evt20, robot_1, medium).
distance_bucket(order_18_evt20, robot_2, medium).
distance_bucket(order_19_evt21, robot_1, near).
distance_bucket(order_19_evt21, robot_2, near).
distance_bucket(order_1_evt1, robot_1, medium).
distance_bucket(order_1_evt1, robot_2, medium).
distance_bucket(order_1_evt3, robot_1, medium).
distance_bucket(order_1_evt3, robot_2, medium).
distance_bucket(order_20_evt22, robot_1, near).
distance_bucket(order_20_evt22, robot_2, near).
distance_bucket(order_21_evt23, robot_1, medium).
distance_bucket(order_21_evt23, robot_2, medium).
distance_bucket(order_22_evt24, robot_1, medium).
distance_bucket(order_22_evt24, robot_2, near).
distance_bucket(order_23_evt25, robot_1, near).
distance_bucket(order_23_evt25, robot_2, near).
distance_bucket(order_2_evt2, robot_1, medium).
distance_bucket(order_2_evt2, robot_2, medium).
distance_bucket(order_2_evt4, robot_1, near).
distance_bucket(order_2_evt4, robot_2, near).
distance_bucket(order_3_evt5, robot_1, medium).
distance_bucket(order_3_evt5, robot_2, near).
distance_bucket(order_4_evt6, robot_1, near).
distance_bucket(order_4_evt6, robot_2, near).
distance_bucket(order_5_evt7, robot_1, medium).
distance_bucket(order_5_evt7, robot_2, medium).
distance_bucket(order_6_evt8, robot_1, medium).
distance_bucket(order_6_evt8, robot_2, medium).
distance_bucket(order_7_evt9, robot_1, medium).
distance_bucket(order_7_evt9, robot_2, near).
distance_bucket(order_8_evt10, robot_1, near).
distance_bucket(order_8_evt10, robot_2, near).
distance_bucket(order_9_evt11, robot_1, medium).
distance_bucket(order_9_evt11, robot_2, medium).
idle(order_10_evt12, robot_1).
idle(order_11_evt13, robot_2).
idle(order_12_evt14, robot_1).
idle(order_13_evt15, robot_1).
idle(order_14_evt16, robot_1).
idle(order_14_evt16, robot_2).
idle(order_15_evt17, robot_1).
idle(order_16_evt18, robot_1).
idle(order_17_evt19, robot_2).
idle(order_18_evt20, robot_1).
idle(order_19_evt21, robot_2).
idle(order_1_evt1, robot_1).
idle(order_1_evt1, robot_2).
idle(order_1_evt3, robot_1).
idle(order_1_evt3, robot_2).
idle(order_20_evt22, robot_2).
idle(order_21_evt23, robot_2).
idle(order_22_evt24, robot_1).
idle(order_23_evt25, robot_2).
idle(order_2_evt2, robot_1).
idle(order_2_evt4, robot_2).
idle(order_3_evt5, robot_1).
idle(order_4_evt6, robot_2).
idle(order_5_evt7, robot_1).
idle(order_6_evt8, robot_2).
idle(order_7_evt9, robot_1).
idle(order_8_evt10, robot_2).
idle(order_9_evt11, robot_2).
most_charged(order_10_evt12, robot_1).
most_charged(order_11_evt13, robot_2).
most_charged(order_12_evt14, robot_1).
most_charged(order_13_evt15, robot_1).
most_charged(order_14_evt16, robot_1).
most_charged(order_14_evt16, robot_2).
most_charged(order_15_evt17, robot_1).
most_charged(order_16_evt18, robot_1).
most_charged(order_17_evt19, robot_2).
most_charged(order_18_evt20, robot_1).
most_charged(order_19_evt21, robot_2).
most_charged(order_1_evt1, robot_1).
most_charged(order_1_evt1, robot_2).
most_charged(order_1_evt3, robot_1).
most_charged(order_1_evt3, robot_2).
most_charged(order_20_evt22, robot_2).
most_charged(order_21_evt23, robot_2).
most_charged(order_22_evt24, robot_1).
most_charged(order_23_evt25, robot_2).
most_charged(order_2_evt2, robot_1).
most_charged(order_2_evt4, robot_2).
most_charged(order_3_evt5, robot_1).
most_charged(order_4_evt6, robot_2).
most_charged(order_5_evt7, robot_1).
most_charged(order_6_evt8, robot_2).
most_charged(order_7_evt9, robot_1).
most_charged(order_8_evt10, robot_2).
most_charged(order_9_evt11, robot_2).
workload_bucket(order_10_evt12, robot_1, zero).
workload_bucket(order_10_evt12, robot_2, zero).
workload_bucket(order_11_evt13, robot_1, zero).
workload_bucket(order_11_evt13, robot_2, zero).
workload_bucket(order_12_evt14, robot_1, zero).
workload_bucket(order_12_evt14, robot_2, zero).
workload_bucket(order_13_evt15, robot_1, zero).
workload_bucket(order_13_evt15, robot_2, zero).
workload_bucket(order_14_evt16, robot_1, zero).
workload_bucket(order_14_evt16, robot_2, zero).
workload_bucket(order_15_evt17, robot_1, zero).
workload_bucket(order_15_evt17, robot_2, zero).
workload_bucket(order_16_evt18, robot_1, zero).
workload_bucket(order_16_evt18, robot_2, zero).
workload_bucket(order_17_evt19, robot_1, zero).
workload_bucket(order_17_evt19, robot_2, zero).
workload_bucket(order_18_evt20, robot_1, zero).
workload_bucket(order_18_evt20, robot_2, zero).
workload_bucket(order_19_evt21, robot_1, zero).
workload_bucket(order_19_evt21, robot_2, zero).
workload_bucket(order_1_evt1, robot_1, zero).
workload_bucket(order_1_evt1, robot_2, zero).
workload_bucket(order_1_evt3, robot_1, zero).
workload_bucket(order_1_evt3, robot_2, zero).
workload_bucket(order_20_evt22, robot_1, zero).
workload_bucket(order_20_evt22, robot_2, zero).
workload_bucket(order_21_evt23, robot_1, zero).
workload_bucket(order_21_evt23, robot_2, zero).
workload_bucket(order_22_evt24, robot_1, zero).
workload_bucket(order_22_evt24, robot_2, zero).
workload_bucket(order_23_evt25, robot_1, zero).
workload_bucket(order_23_evt25, robot_2, zero).
workload_bucket(order_2_evt2, robot_1, zero).
workload_bucket(order_2_evt2, robot_2, zero).
workload_bucket(order_2_evt4, robot_1, zero).
workload_bucket(order_2_evt4, robot_2, zero).
workload_bucket(order_3_evt5, robot_1, zero).
workload_bucket(order_3_evt5, robot_2, zero).
workload_bucket(order_4_evt6, robot_1, zero).
workload_bucket(order_4_evt6, robot_2, zero).
workload_bucket(order_5_evt7, robot_1, zero).
workload_bucket(order_5_evt7, robot_2, zero).
workload_bucket(order_6_evt8, robot_1, zero).
workload_bucket(order_6_evt8, robot_2, zero).
workload_bucket(order_7_evt9, robot_1, zero).
workload_bucket(order_7_evt9, robot_2, zero).
workload_bucket(order_8_evt10, robot_1, zero).
workload_bucket(order_8_evt10, robot_2, zero).
workload_bucket(order_9_evt11, robot_1, zero).
workload_bucket(order_9_evt11, robot_2, zero).