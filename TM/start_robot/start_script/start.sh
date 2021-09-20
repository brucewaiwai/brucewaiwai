#!/bin/bash
sleep 5
xfce4-terminal -T master -e "bash -iC 'sleep 3; echo start!; $SHELL'" \
        --tab -T roscore -e "bash -ic ' roscore; $SHELL'" \
        --tab -T sick_scan -e "bash -ic ' roslaunch tm_controller sick_scan.launch --wait; $SHELL'" \
        --tab -T s1_scan -e "bash -ic 'sleep 3; roslaunch tm_controller rplidar_s1_tcp.launch --wait; $SHELL'" \
        --tab -T laser_filter -e "bash -ic ' roslaunch tm_controller laser_filter.launch --wait; $SHELL'" \
        --tab -T laser_merger -e "bash -ic 'sleep 5; roslaunch tm_controller laserscan_multi_merger.launch --wait; $SHELL'" \
        --tab -T joy -e "bash -ic 'sleep 2; roslaunch joystick_control jsc.launch --wait; $SHELL'" \
        --tab -T ekf_se -e "bash -ic 'sleep 10; roslaunch turn_on_wheeltec_robot robot_pose_ekf.launch --wait; $SHELL'" \
        --tab -T Arduino -e "bash -ic 'sleep 10; roslaunch tm_controller arduino.launch --wait; $SHELL'" \
        --tab -T Auto_charging -e "bash -ic 'sleep 10; roslaunch auto_parking auto_park.launch --wait; $SHELL'" &
xfce4-terminal -T master -e "bash -iC 'echo start!; $SHELL'" \
        --tab -T wheeltec_base -e "bash -ic 'sleep 3; roslaunch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch --wait; $SHELL'" \
        --tab -T nav -e "bash -ic 'sleep 10; roslaunch turn_on_wheeltec_robot dwa_local_planner.launch --wait; $SHELL'" \
        --tab -T map -e "bash -ic 'sleep 10; roslaunch turn_on_wheeltec_robot amcl_map.launch --wait; $SHELL'" \
        --tab -T delta -e "bash -ic ' roslaunch tm_controller delta.launch --wait; $SHELL'" \
        --tab -T tm_controller -e "bash -ic 'roslaunch tm_controller tm_controller.launch --wait; $SHELL'" \
        --tab -T cam -e "bash -ic 'sleep 10; roslaunch tm_controller pointcloud_to_laserscan.launch --wait; $SHELL'" \
	# --tab -T rosbridge -e "bash -ic 'roslaunch rosbridge_server rosbridge_websocket.launch --wait; $SHELL'" \
        # --tab -T tm_bridge -e "bash -ic 'roslaunch tm_modules tm_bridge.launch --wait; $SHELL'" \
        # --tab -T transport -e "bash -ic 'roslaunch tm_modules transport.launch --wait; $SHELL'" \
        # --tab -T sick_scan -e "bash -ic ' roslaunch tm_controller sick_scan.launch --wait; $SHELL'" \
        # --tab -T sick_scan1 -e "bash -ic ' roslaunch sick_scan scan_1.launch --wait; $SHELL'" \
        # --tab -T sick_scan2 -e "bash -ic ' roslaunch sick_scan scan_2.launch --wait; $SHELL'" \
        # --tab -T sick_scan3 -e "bash -ic ' roslaunch sick_scan scan_3.launch --wait; $SHELL'" \
        # --tab -T sick_scan4 -e "bash -ic ' roslaunch sick_scan scan_4.launch --wait; $SHELL'" \
        # --tab -T realsense -e "bash -ic ' roslaunch realsense2_camera rs_camera.launch --wait; $SHELL'" \
        # --tab -T rplidar -e "bash -ic ' roslaunch rplidar_ros rplidar_a3.launch --wait; $SHELL'" \
        # --tab -T imu_tool -e "bash -ic 'sleep 2; roslaunch imu_transformer ned_to_enu.launch --wait; $SHELL'" \
        # --tab -T ekf_se -e "bash -ic 'roslaunch robot_localization ekf_template.launch --wait; $SHELL'" \
        # --tab -T laser_merge1 -e "bash -ic 'sleep 2; roslaunch ira_laser_tools sick_merger.launch --wait; $SHELL'" \
        # --tab -T laser_merge2 -e "bash -ic 'sleep 5; roslaunch ira_laser_tools sick_s1_merger.launch --wait; $SHELL'" \



        
