#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
xhost +
docker run -it --rm \
  --name ov-env \
  --network=host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  ov-env:latest \
  bash -lc "
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    cd /root/workspace/catkin_ws_ov
    source install/setup.bash
    ros2 run ov_msckf run_subscribe_msckf src/open_vins/config/picam360/estimator_config.yaml --ros-args -r __ns:=/ov_msckf
  "