
mkdir -p ~/workspaces/isaac_ros-dev/src
export ISAAC_ROS_WS=${HOME}/workspaces/isaac_ros-dev
cd $ISAAC_ROS_WS/src

#[preparation for test data]
sudo apt-get install -y git-lfs
git lfs install --skip-repo

git clone --branch v2.1.0 --depth 1 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
git clone --branch v2.1.0 --depth 1 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git

#[preparation for run_dev.sh]
sudo apt-get update
sudo apt-get install -y ca-certificates curl gnupg
#create folder
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg
echo "deb [arch="$(dpkg --print-architecture)" signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
     "$(. /etc/os-release && echo "$VERSION_CODENAME")" stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update
sudo apt-get install -y docker-buildx-plugin

bash ./isaac_ros_common/scripts/run_dev.sh ${ISAAC_ROS_WS}

#[internal container]
#sudo apt-get install -y ros-humble-isaac-ros-visual-slam
#ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam.launch.py

#rviz2 -d src/isaac_ros_visual_slam/isaac_ros_visual_slam/rviz/default.cfg.rviz

#ros2 bag play src/isaac_ros_visual_slam/isaac_ros_visual_slam/test/test_cases/rosbags/small_pol_test/