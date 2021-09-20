#!/bin/bash
xfce4-terminal -T master -e "bash -iC 'echo start!; $SHELL'" \
        --tab -T roscore -e "bash -ic 'roscore; $SHELL'" \
        --tab -T sick_scan -e "bash -ic ' roslaunch tm_controller sick_scan.launch --wait; $SHELL'" \
        --tab -T s1_scan -e "bash -ic ' roslaunch tm_controller rplidar_s1_tcp.launch --wait; $SHELL'" \
        --tab -T laser_filter -e "bash -ic ' roslaunch tm_controller laser_filter.launch --wait; $SHELL'" \
        --tab -T joy -e "bash -ic 'sleep 2; roslaunch joystick_control jsc.launch --wait; $SHELL'" \
        --tab -T wheeltec_base -e "bash -ic 'sleep 3; roslaunch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch --wait; $SHELL'" \
        --tab -T cartographer -e "bash -ic 'sleep 10; roslaunch tm_controller mapping.launch --wait; $SHELL'" \
        --tab -T Arduino -e "bash -ic 'sleep 10; roslaunch tm_controller arduino.launch --wait; $SHELL'" \
        --tab -T tm_controller -e "bash -ic 'roslaunch tm_controller tm_controller.launch --wait; $SHELL'" \
        # --tab -T s1_scan -e "bash -ic ' roslaunch rplidar_ros rplidar_s1_tcp.launch --wait; $SHELL'" \
        # --tab -T rplidar -e "bash -ic ' roslaunch rplidar_ros rplidar_a3.launch --wait; $SHELL'" \

        # --tab -T realsense -e "bash -ic ' roslaunch realsense2_camera rs_camera.launch --wait; $SHELL'" \


        
