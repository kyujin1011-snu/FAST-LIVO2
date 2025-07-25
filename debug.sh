#!/bin/bash

cd ~/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=Debug
gdb --args /home/jmseo1204/catkin_ws/devel/lib/fast_livo/fastlivo_mapping