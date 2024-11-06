FROM arm64v8/ubuntu:20.04

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic

# Create user arguments
ARG USERNAME=picam360
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Install dependencies
RUN apt-get update && apt-get install -y \
    nano \
    curl \
    lsb-release \
    gnupg2 \
    python3-pip \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# Create user and group
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Setup ROS 1 repository
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

# Install ROS Noetic and build tools
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-desktop-full \
    ros-${ROS_DISTRO}-robot-localization \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential \
    python3-catkin-tools \
    # X11関連パッケージ
    libxcb1-dev \
    libqt5x11extras5 \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*

# Install pyproj using pip
RUN pip3 install pyproj

# Initialize rosdep
RUN rosdep init && rosdep update

# Set up ROS workspace
ENV WORKSPACE=/home/$USERNAME/workspace/${ROS_DISTRO}_ws
ENV ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:${WORKSPACE}/src/wheel_odometry

# Create workspace directory and set permissions
RUN mkdir -p ${WORKSPACE} \
    && chown -R $USERNAME:$USERNAME /home/$USERNAME

# Setup environment for user
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:${WORKSPACE}/src/wheel_odometry" >> /home/$USERNAME/.bashrc

# Add user to video group for GUI applications
RUN usermod -a -G video $USERNAME

# Switch to new user
USER $USERNAME

# Set the working directory
WORKDIR ${WORKSPACE}

# Create entrypoint script
RUN echo '#!/bin/bash\n\
source /opt/ros/'${ROS_DISTRO}'/setup.bash\n\
source $HOME/.bashrc\n\
exec "$@"' > /home/$USERNAME/entrypoint.sh \
    && chmod +x /home/$USERNAME/entrypoint.sh

# 環境変数としてENTRYPOINTのパスを設定
ENV ENTRYPOINT_PATH=/home/${USERNAME}/entrypoint.sh

# 環境変数を使用してENTRYPOINTを設定
ENTRYPOINT ["sh", "-c", "exec ${ENTRYPOINT_PATH}"]
CMD ["/bin/bash"]