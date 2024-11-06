cd /workspaces/isaac_ros-dev
rm -rf build/auto-drive-file-loader install/auto-drive-file-loader
colcon build --packages-select auto-drive-file-loader
source install/setup.bash