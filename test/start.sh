#!/bin/bash
cd ~/galaxea/install/share/startup_config/script/
./robot_startup.sh boot ../session.d/ACTStandard/R1LITEBody.d
cd ~/galaxea
source install/setup.bash
roslaunch rds_ros rds_check.launch
