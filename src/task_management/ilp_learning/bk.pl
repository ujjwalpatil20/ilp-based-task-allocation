:- dynamic battery_bucket/3, distance_bucket/3, workload_bucket/3, idle/2, busy/2, closest/2, most_charged/2.
battery_bucket(order_10_evt5, robot_4, medium).
battery_bucket(order_11_evt6, robot_1, medium).
battery_bucket(order_12_evt7, robot_2, medium).
battery_bucket(order_13_evt8, robot_3, high).
battery_bucket(order_13_evt8, robot_4, high).
battery_bucket(order_14_evt9, robot_4, high).
battery_bucket(order_15_evt10, robot_2, low).
battery_bucket(order_16_evt11, robot_3, high).
battery_bucket(order_17_evt12, robot_2, medium).
battery_bucket(order_18_evt13, robot_1, high).
battery_bucket(order_19_evt14, robot_4, high).
battery_bucket(order_20_evt15, robot_2, high).
battery_bucket(order_21_evt16, robot_3, medium).
battery_bucket(order_22_evt17, robot_2, high).
battery_bucket(order_23_evt18, robot_1, high).
battery_bucket(order_24_evt19, robot_4, medium).
battery_bucket(order_25_evt20, robot_3, medium).
battery_bucket(order_26_evt21, robot_1, medium).
battery_bucket(order_27_evt22, robot_4, medium).
battery_bucket(order_28_evt23, robot_3, high).
battery_bucket(order_29_evt24, robot_2, medium).
battery_bucket(order_30_evt25, robot_1, medium).
battery_bucket(order_31_evt26, robot_2, medium).
battery_bucket(order_32_evt27, robot_3, high).
battery_bucket(order_33_evt28, robot_4, high).
battery_bucket(order_34_evt29, robot_1, high).
battery_bucket(order_35_evt30, robot_3, high).
battery_bucket(order_36_evt31, robot_2, low).
battery_bucket(order_37_evt32, robot_1, high).
battery_bucket(order_38_evt33, robot_4, high).
battery_bucket(order_39_evt34, robot_2, high).
battery_bucket(order_40_evt35, robot_3, medium).
battery_bucket(order_41_evt36, robot_2, high).
battery_bucket(order_42_evt37, robot_4, high).
battery_bucket(order_43_evt38, robot_1, medium).
battery_bucket(order_43_evt38, robot_2, high).
battery_bucket(order_43_evt38, robot_3, low).
battery_bucket(order_43_evt38, robot_4, medium).
battery_bucket(order_44_evt39, robot_2, high).
battery_bucket(order_44_evt39, robot_3, low).
battery_bucket(order_44_evt39, robot_4, medium).
battery_bucket(order_45_evt40, robot_3, low).
battery_bucket(order_45_evt40, robot_4, medium).
battery_bucket(order_46_evt41, robot_4, medium).
battery_bucket(order_47_evt42, robot_4, low).
battery_bucket(order_48_evt43, robot_3, high).
battery_bucket(order_49_evt44, robot_1, low).
battery_bucket(order_50_evt45, robot_4, high).
battery_bucket(order_51_evt46, robot_2, medium).
battery_bucket(order_52_evt47, robot_3, high).
battery_bucket(order_53_evt48, robot_1, high).
battery_bucket(order_54_evt49, robot_4, high).
battery_bucket(order_55_evt50, robot_2, high).
battery_bucket(order_56_evt51, robot_1, high).
battery_bucket(order_57_evt52, robot_3, medium).
battery_bucket(order_58_evt53, robot_4, high).
battery_bucket(order_59_evt54, robot_2, high).
battery_bucket(order_6_evt1, robot_3, medium).
battery_bucket(order_7_evt2, robot_1, medium).
battery_bucket(order_8_evt3, robot_2, high).
battery_bucket(order_9_evt4, robot_3, low).
closest(order_10_evt5, robot_4).
closest(order_11_evt6, robot_1).
closest(order_12_evt7, robot_2).
closest(order_13_evt8, robot_3).
closest(order_14_evt9, robot_4).
closest(order_15_evt10, robot_2).
closest(order_16_evt11, robot_3).
closest(order_17_evt12, robot_2).
closest(order_18_evt13, robot_1).
closest(order_19_evt14, robot_4).
closest(order_20_evt15, robot_2).
closest(order_21_evt16, robot_3).
closest(order_22_evt17, robot_2).
closest(order_23_evt18, robot_1).
closest(order_24_evt19, robot_4).
closest(order_25_evt20, robot_3).
closest(order_26_evt21, robot_1).
closest(order_27_evt22, robot_4).
closest(order_28_evt23, robot_3).
closest(order_29_evt24, robot_2).
closest(order_30_evt25, robot_1).
closest(order_31_evt26, robot_2).
closest(order_32_evt27, robot_3).
closest(order_33_evt28, robot_4).
closest(order_34_evt29, robot_1).
closest(order_35_evt30, robot_3).
closest(order_36_evt31, robot_2).
closest(order_37_evt32, robot_1).
closest(order_38_evt33, robot_4).
closest(order_39_evt34, robot_2).
closest(order_40_evt35, robot_3).
closest(order_41_evt36, robot_2).
closest(order_42_evt37, robot_4).
closest(order_43_evt38, robot_1).
closest(order_44_evt39, robot_2).
closest(order_45_evt40, robot_3).
closest(order_46_evt41, robot_4).
closest(order_47_evt42, robot_4).
closest(order_48_evt43, robot_3).
closest(order_49_evt44, robot_1).
closest(order_50_evt45, robot_4).
closest(order_51_evt46, robot_2).
closest(order_52_evt47, robot_3).
closest(order_53_evt48, robot_1).
closest(order_54_evt49, robot_4).
closest(order_55_evt50, robot_2).
closest(order_56_evt51, robot_1).
closest(order_57_evt52, robot_3).
closest(order_58_evt53, robot_4).
closest(order_59_evt54, robot_2).
closest(order_6_evt1, robot_3).
closest(order_7_evt2, robot_1).
closest(order_8_evt3, robot_2).
closest(order_9_evt4, robot_3).
distance_bucket(order_10_evt5, robot_4, near).
distance_bucket(order_11_evt6, robot_1, medium).
distance_bucket(order_12_evt7, robot_2, near).
distance_bucket(order_13_evt8, robot_3, near).
distance_bucket(order_13_evt8, robot_4, medium).
distance_bucket(order_14_evt9, robot_4, medium).
distance_bucket(order_15_evt10, robot_2, near).
distance_bucket(order_16_evt11, robot_3, medium).
distance_bucket(order_17_evt12, robot_2, near).
distance_bucket(order_18_evt13, robot_1, near).
distance_bucket(order_19_evt14, robot_4, medium).
distance_bucket(order_20_evt15, robot_2, medium).
distance_bucket(order_21_evt16, robot_3, medium).
distance_bucket(order_22_evt17, robot_2, medium).
distance_bucket(order_23_evt18, robot_1, medium).
distance_bucket(order_24_evt19, robot_4, near).
distance_bucket(order_25_evt20, robot_3, medium).
distance_bucket(order_26_evt21, robot_1, near).
distance_bucket(order_27_evt22, robot_4, near).
distance_bucket(order_28_evt23, robot_3, medium).
distance_bucket(order_29_evt24, robot_2, medium).
distance_bucket(order_30_evt25, robot_1, near).
distance_bucket(order_31_evt26, robot_2, medium).
distance_bucket(order_32_evt27, robot_3, near).
distance_bucket(order_33_evt28, robot_4, medium).
distance_bucket(order_34_evt29, robot_1, near).
distance_bucket(order_35_evt30, robot_3, near).
distance_bucket(order_36_evt31, robot_2, medium).
distance_bucket(order_37_evt32, robot_1, medium).
distance_bucket(order_38_evt33, robot_4, medium).
distance_bucket(order_39_evt34, robot_2, near).
distance_bucket(order_40_evt35, robot_3, medium).
distance_bucket(order_41_evt36, robot_2, medium).
distance_bucket(order_42_evt37, robot_4, near).
distance_bucket(order_43_evt38, robot_1, medium).
distance_bucket(order_43_evt38, robot_2, medium).
distance_bucket(order_43_evt38, robot_3, medium).
distance_bucket(order_43_evt38, robot_4, medium).
distance_bucket(order_44_evt39, robot_2, medium).
distance_bucket(order_44_evt39, robot_3, medium).
distance_bucket(order_44_evt39, robot_4, medium).
distance_bucket(order_45_evt40, robot_3, medium).
distance_bucket(order_45_evt40, robot_4, medium).
distance_bucket(order_46_evt41, robot_4, near).
distance_bucket(order_47_evt42, robot_4, near).
distance_bucket(order_48_evt43, robot_3, near).
distance_bucket(order_49_evt44, robot_1, near).
distance_bucket(order_50_evt45, robot_4, medium).
distance_bucket(order_51_evt46, robot_2, medium).
distance_bucket(order_52_evt47, robot_3, medium).
distance_bucket(order_53_evt48, robot_1, near).
distance_bucket(order_54_evt49, robot_4, near).
distance_bucket(order_55_evt50, robot_2, near).
distance_bucket(order_56_evt51, robot_1, medium).
distance_bucket(order_57_evt52, robot_3, near).
distance_bucket(order_58_evt53, robot_4, near).
distance_bucket(order_59_evt54, robot_2, near).
distance_bucket(order_6_evt1, robot_3, medium).
distance_bucket(order_7_evt2, robot_1, near).
distance_bucket(order_8_evt3, robot_2, medium).
distance_bucket(order_9_evt4, robot_3, medium).
idle(order_10_evt5, robot_4).
idle(order_11_evt6, robot_1).
idle(order_12_evt7, robot_2).
idle(order_13_evt8, robot_3).
idle(order_13_evt8, robot_4).
idle(order_14_evt9, robot_4).
idle(order_15_evt10, robot_2).
idle(order_16_evt11, robot_3).
idle(order_17_evt12, robot_2).
idle(order_18_evt13, robot_1).
idle(order_19_evt14, robot_4).
idle(order_20_evt15, robot_2).
idle(order_21_evt16, robot_3).
idle(order_22_evt17, robot_2).
idle(order_23_evt18, robot_1).
idle(order_24_evt19, robot_4).
idle(order_25_evt20, robot_3).
idle(order_26_evt21, robot_1).
idle(order_27_evt22, robot_4).
idle(order_28_evt23, robot_3).
idle(order_29_evt24, robot_2).
idle(order_30_evt25, robot_1).
idle(order_31_evt26, robot_2).
idle(order_32_evt27, robot_3).
idle(order_33_evt28, robot_4).
idle(order_34_evt29, robot_1).
idle(order_35_evt30, robot_3).
idle(order_36_evt31, robot_2).
idle(order_37_evt32, robot_1).
idle(order_38_evt33, robot_4).
idle(order_39_evt34, robot_2).
idle(order_40_evt35, robot_3).
idle(order_41_evt36, robot_2).
idle(order_42_evt37, robot_4).
idle(order_43_evt38, robot_1).
idle(order_43_evt38, robot_2).
idle(order_43_evt38, robot_3).
idle(order_43_evt38, robot_4).
idle(order_44_evt39, robot_2).
idle(order_44_evt39, robot_3).
idle(order_44_evt39, robot_4).
idle(order_45_evt40, robot_3).
idle(order_45_evt40, robot_4).
idle(order_46_evt41, robot_4).
idle(order_47_evt42, robot_4).
idle(order_48_evt43, robot_3).
idle(order_49_evt44, robot_1).
idle(order_50_evt45, robot_4).
idle(order_51_evt46, robot_2).
idle(order_52_evt47, robot_3).
idle(order_53_evt48, robot_1).
idle(order_54_evt49, robot_4).
idle(order_55_evt50, robot_2).
idle(order_56_evt51, robot_1).
idle(order_57_evt52, robot_3).
idle(order_58_evt53, robot_4).
idle(order_59_evt54, robot_2).
idle(order_6_evt1, robot_3).
idle(order_7_evt2, robot_1).
idle(order_8_evt3, robot_2).
idle(order_9_evt4, robot_3).
most_charged(order_10_evt5, robot_4).
most_charged(order_11_evt6, robot_1).
most_charged(order_12_evt7, robot_2).
most_charged(order_13_evt8, robot_4).
most_charged(order_14_evt9, robot_4).
most_charged(order_15_evt10, robot_2).
most_charged(order_16_evt11, robot_3).
most_charged(order_17_evt12, robot_2).
most_charged(order_18_evt13, robot_1).
most_charged(order_19_evt14, robot_4).
most_charged(order_20_evt15, robot_2).
most_charged(order_21_evt16, robot_3).
most_charged(order_22_evt17, robot_2).
most_charged(order_23_evt18, robot_1).
most_charged(order_24_evt19, robot_4).
most_charged(order_25_evt20, robot_3).
most_charged(order_26_evt21, robot_1).
most_charged(order_27_evt22, robot_4).
most_charged(order_28_evt23, robot_3).
most_charged(order_29_evt24, robot_2).
most_charged(order_30_evt25, robot_1).
most_charged(order_31_evt26, robot_2).
most_charged(order_32_evt27, robot_3).
most_charged(order_33_evt28, robot_4).
most_charged(order_34_evt29, robot_1).
most_charged(order_35_evt30, robot_3).
most_charged(order_36_evt31, robot_2).
most_charged(order_37_evt32, robot_1).
most_charged(order_38_evt33, robot_4).
most_charged(order_39_evt34, robot_2).
most_charged(order_40_evt35, robot_3).
most_charged(order_41_evt36, robot_2).
most_charged(order_42_evt37, robot_4).
most_charged(order_43_evt38, robot_2).
most_charged(order_44_evt39, robot_2).
most_charged(order_45_evt40, robot_4).
most_charged(order_46_evt41, robot_4).
most_charged(order_47_evt42, robot_4).
most_charged(order_48_evt43, robot_3).
most_charged(order_49_evt44, robot_1).
most_charged(order_50_evt45, robot_4).
most_charged(order_51_evt46, robot_2).
most_charged(order_52_evt47, robot_3).
most_charged(order_53_evt48, robot_1).
most_charged(order_54_evt49, robot_4).
most_charged(order_55_evt50, robot_2).
most_charged(order_56_evt51, robot_1).
most_charged(order_57_evt52, robot_3).
most_charged(order_58_evt53, robot_4).
most_charged(order_59_evt54, robot_2).
most_charged(order_6_evt1, robot_3).
most_charged(order_7_evt2, robot_1).
most_charged(order_8_evt3, robot_2).
most_charged(order_9_evt4, robot_3).
workload_bucket(order_10_evt5, robot_4, low).
workload_bucket(order_11_evt6, robot_1, zero).
workload_bucket(order_12_evt7, robot_2, zero).
workload_bucket(order_13_evt8, robot_3, zero).
workload_bucket(order_13_evt8, robot_4, low).
workload_bucket(order_14_evt9, robot_4, low).
workload_bucket(order_15_evt10, robot_2, zero).
workload_bucket(order_16_evt11, robot_3, zero).
workload_bucket(order_17_evt12, robot_2, zero).
workload_bucket(order_18_evt13, robot_1, zero).
workload_bucket(order_19_evt14, robot_4, low).
workload_bucket(order_20_evt15, robot_2, zero).
workload_bucket(order_21_evt16, robot_3, zero).
workload_bucket(order_22_evt17, robot_2, zero).
workload_bucket(order_23_evt18, robot_1, zero).
workload_bucket(order_24_evt19, robot_4, low).
workload_bucket(order_25_evt20, robot_3, zero).
workload_bucket(order_26_evt21, robot_1, zero).
workload_bucket(order_27_evt22, robot_4, low).
workload_bucket(order_28_evt23, robot_3, zero).
workload_bucket(order_29_evt24, robot_2, zero).
workload_bucket(order_30_evt25, robot_1, zero).
workload_bucket(order_31_evt26, robot_2, zero).
workload_bucket(order_32_evt27, robot_3, zero).
workload_bucket(order_33_evt28, robot_4, low).
workload_bucket(order_34_evt29, robot_1, zero).
workload_bucket(order_35_evt30, robot_3, zero).
workload_bucket(order_36_evt31, robot_2, zero).
workload_bucket(order_37_evt32, robot_1, zero).
workload_bucket(order_38_evt33, robot_4, low).
workload_bucket(order_39_evt34, robot_2, zero).
workload_bucket(order_40_evt35, robot_3, zero).
workload_bucket(order_41_evt36, robot_2, zero).
workload_bucket(order_42_evt37, robot_4, low).
workload_bucket(order_43_evt38, robot_1, zero).
workload_bucket(order_43_evt38, robot_2, zero).
workload_bucket(order_43_evt38, robot_3, zero).
workload_bucket(order_43_evt38, robot_4, low).
workload_bucket(order_44_evt39, robot_2, zero).
workload_bucket(order_44_evt39, robot_3, zero).
workload_bucket(order_44_evt39, robot_4, low).
workload_bucket(order_45_evt40, robot_3, zero).
workload_bucket(order_45_evt40, robot_4, low).
workload_bucket(order_46_evt41, robot_4, low).
workload_bucket(order_47_evt42, robot_4, low).
workload_bucket(order_48_evt43, robot_3, zero).
workload_bucket(order_49_evt44, robot_1, zero).
workload_bucket(order_50_evt45, robot_4, low).
workload_bucket(order_51_evt46, robot_2, zero).
workload_bucket(order_52_evt47, robot_3, zero).
workload_bucket(order_53_evt48, robot_1, zero).
workload_bucket(order_54_evt49, robot_4, low).
workload_bucket(order_55_evt50, robot_2, zero).
workload_bucket(order_56_evt51, robot_1, zero).
workload_bucket(order_57_evt52, robot_3, zero).
workload_bucket(order_58_evt53, robot_4, low).
workload_bucket(order_59_evt54, robot_2, zero).
workload_bucket(order_6_evt1, robot_3, zero).
workload_bucket(order_7_evt2, robot_1, zero).
workload_bucket(order_8_evt3, robot_2, zero).
workload_bucket(order_9_evt4, robot_3, zero).