#!/bin/sh
catkin_make && source devel/setup.sh && roslaunch simple_bringup game.launch
