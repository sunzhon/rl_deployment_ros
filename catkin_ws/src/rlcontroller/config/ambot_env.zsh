#!/bin/zsh
export ROS_MASTER_URI=http://localhost:11311
#export ROS_MASTER_URI=http://10.88.104.80:11311
#export ROS_IP=10.88.104.80
#export ROS_MASTER_URI=http://192.168.68.1:11311
#export ROS_MASTER_URI=http://192.168.0.1:11311
#export ROS_IP=192.168.0.1
export ROS_HOSTNAME=$(hostname)
export ROSLAUNCH_SSH_UNKNOWN=1
#source ~/workspace/parkour/catkin_ws/devel/setup.zsh

exec "$@"
