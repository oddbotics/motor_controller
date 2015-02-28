#!/bin/bash
source /opt/ros/indigo/setup.bash
source /home/ubuntu/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://odroid:11311
export ROS_HOSTNAME=motor-1

sleep 2;

address=$(ip addr show eth0 | grep -o '10.0.[[:digit:]].[[:digit:]]' | head -1 |
awk '{
	split($1, a, ".")
	subnet = a[3]
	print "object_" subnet
}')

roslaunch motor_controller motor.launch position:=$address type:=dc_motor
