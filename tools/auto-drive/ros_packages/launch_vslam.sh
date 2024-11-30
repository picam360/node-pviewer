#!/bin/bash

BASE_NAME="vslam"
CONTAINER_NAME="$BASE_NAME-container"

# Re-use existing container.
if [ "$(docker ps -a --quiet --filter status=running --filter name=$CONTAINER_NAME)" ]; then
    echo "Attaching to running container: $CONTAINER_NAME"
    docker exec -i -t -u admin --workdir /workspaces/isaac_ros-dev $CONTAINER_NAME /bin/bash $@
    exit 0
fi

DOCKER_ARGS+=("-v /tmp/.X11-unix:/tmp/.X11-unix")
DOCKER_ARGS+=("-v $HOME/.Xauthority:/home/admin/.Xauthority:rw")
DOCKER_ARGS+=("-e DISPLAY")
DOCKER_ARGS+=("-e NVIDIA_VISIBLE_DEVICES=all")
DOCKER_ARGS+=("-e NVIDIA_DRIVER_CAPABILITIES=all")
DOCKER_ARGS+=("-e FASTRTPS_DEFAULT_PROFILES_FILE=/usr/local/share/middleware_profiles/rtps_udp_profile.xml")
DOCKER_ARGS+=("-e ROS_DOMAIN_ID")
DOCKER_ARGS+=("-e USER")

DOCKER_ARGS+=("-v /usr/bin/tegrastats:/usr/bin/tegrastats")
DOCKER_ARGS+=("-v /tmp/argus_socket:/tmp/argus_socket")
DOCKER_ARGS+=("-v /usr/local/cuda-11.4/targets/aarch64-linux/lib/libcusolver.so.11:/usr/local/cuda-11.4/targets/aarch64-linux/lib/libcusolver.so.11")
DOCKER_ARGS+=("-v /usr/local/cuda-11.4/targets/aarch64-linux/lib/libcusparse.so.11:/usr/local/cuda-11.4/targets/aarch64-linux/lib/libcusparse.so.11")
DOCKER_ARGS+=("-v /usr/local/cuda-11.4/targets/aarch64-linux/lib/libcurand.so.10:/usr/local/cuda-11.4/targets/aarch64-linux/lib/libcurand.so.10")
DOCKER_ARGS+=("-v /usr/local/cuda-11.4/targets/aarch64-linux/lib/libcufft.so.10:/usr/local/cuda-11.4/targets/aarch64-linux/lib/libcufft.so.10")
DOCKER_ARGS+=("-v /usr/local/cuda-11.4/targets/aarch64-linux/lib/libnvToolsExt.so:/usr/local/cuda-11.4/targets/aarch64-linux/lib/libnvToolsExt.so")
DOCKER_ARGS+=("-v /usr/local/cuda-11.4/targets/aarch64-linux/lib/libcupti.so.11.4:/usr/local/cuda-11.4/targets/aarch64-linux/lib/libcupti.so.11.4")
DOCKER_ARGS+=("-v /usr/local/cuda-11.4/targets/aarch64-linux/lib/libcudla.so.1:/usr/local/cuda-11.4/targets/aarch64-linux/lib/libcudla.so.1")
DOCKER_ARGS+=("-v /usr/local/cuda-11.4/targets/aarch64-linux/include/nvToolsExt.h:/usr/local/cuda-11.4/targets/aarch64-linux/include/nvToolsExt.h")
DOCKER_ARGS+=("-v /usr/lib/aarch64-linux-gnu/tegra:/usr/lib/aarch64-linux-gnu/tegra")
DOCKER_ARGS+=("-v /usr/src/jetson_multimedia_api:/usr/src/jetson_multimedia_api")
DOCKER_ARGS+=("-v /opt/nvidia/nsight-systems-cli:/opt/nvidia/nsight-systems-cli")
DOCKER_ARGS+=("--pid=host")
DOCKER_ARGS+=("-v /opt/nvidia/vpi2:/opt/nvidia/vpi2")
DOCKER_ARGS+=("-v /usr/share/vpi2:/usr/share/vpi2")

ISAAC_ROS_DEV_DIR=$(pwd)

# Run container from image
echo "Running $CONTAINER_NAME"
docker run -it --rm \
    --privileged \
    --network host \
    ${DOCKER_ARGS[@]} \
    -v $ISAAC_ROS_DEV_DIR:/workspaces/isaac_ros-dev \
    -v /dev/*:/dev/* \
    -v /etc/localtime:/etc/localtime:ro \
    --name "$CONTAINER_NAME" \
    --runtime nvidia \
    --user="admin" \
    --entrypoint /usr/local/bin/scripts/workspace-entrypoint.sh \
    --workdir /workspaces/isaac_ros-dev \
    $@ \
    $BASE_NAME \
    /bin/bash

#NOTE:
#$ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam.launch.py
#$rviz2 -d src/isaac_ros_visual_slam/isaac_ros_visual_slam/rviz/default.cfg.rviz
