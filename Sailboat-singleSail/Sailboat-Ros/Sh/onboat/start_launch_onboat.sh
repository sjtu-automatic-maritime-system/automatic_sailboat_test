#!/usr/bin/env bash
nohup roslaunch sailboat_launch set_environment_onboat.launch &
sleep 2s
nohup roslaunch sailboat_launch start_tf_tree_onboat.launch &
sleep 3s
nohup roslaunch sailboat_launch start_perception.launch &
sleep 5s
nohup rosrun sensor_process kalman4.py &
sleep 5s
# nohup rosrun self_checking self_checking &
# sleep 2s
echo "start launch onboat"
rosnode list
