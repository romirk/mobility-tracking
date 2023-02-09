#!/bin/bash

export ROS_MASTER_URI=http://minichungus.lan:11311

source /opt/ros/melodic/setup.bash
source /mobility-tracking/sbx_ws/devel/setup.bash

exec "$@"