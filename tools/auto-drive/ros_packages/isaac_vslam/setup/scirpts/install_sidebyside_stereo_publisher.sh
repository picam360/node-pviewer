cd /workspaces/isaac_ros-dev
rm -rf build/sidebyside_stereo_publisher install/sidebyside_stereo_publisher
colcon build --packages-select sidebyside_stereo_publisher
source install/setup.bash