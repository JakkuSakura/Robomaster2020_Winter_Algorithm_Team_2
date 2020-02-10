#!/bin/bash
catkin_make && source devel/setup.bash && roslaunch simple_bringup simulator.launch
