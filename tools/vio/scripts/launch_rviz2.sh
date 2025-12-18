#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
xhost +
docker run -it --rm \
  --name ov-rviz \
  --network=host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  ov-env:latest \
  bash -lc "
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    cd /root/workspace/catkin_ws_ov
    source install/setup.bash
    rviz2 -d src/open_vins/ov_msckf/launch/display_ros2.rviz
  "