# Use ROS 2 Iron as the base image
FROM ros:iron

# Set the working environment to root
USER root

# Update APT and install necessary packages
RUN apt-get update && \
    apt-get install -y \
    sudo \
    curl \
    python3-pip \
    python3-venv \
    build-essential \
    wget qtbase5-dev qtchooser qt5-qmake qtbase5-dev-tools \
    python3-rosdep \
    python3-vcstool \
    python3-colcon-common-extensions

# Install Open RMF
RUN apt-get update && \
    apt-get install -y ros-iron-rmf-dev ros-iron-rviz2

# Install Python dependencies
RUN pip3 install --no-cache-dir python-socketio==5.7.2 nudged paho-mqtt networkx pipenv

# Create ROS workspace and necessary directories
RUN mkdir -p /rmf_ws/src && \
    cd /rmf_ws/src && \
    git clone https://github.com/mrceki/hamal_rmf.git -b development_ros2 && \
    git clone https://github.com/mrceki/vda5050_fleet_adapter.git && \
    git clone https://github.com/mrceki/rmf-web.git -b test && \
    git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.7.x

# Resolve ROS dependencies
RUN cd /rmf_ws && \
    rosdep update && \
    sudo rosdep fix-permissions && \
    rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO \
    --skip-keys roscpp \
    --skip-keys actionlib \
    --skip-keys rviz \
    --skip-keys catkin \
    --skip-keys move_base \
    --skip-keys amcl \
    --skip-keys turtlebot3_navigation \
    --skip-keys turtlebot3_bringup \
    --skip-keys turtlebot3_gazebo \
    --skip-keys move_base_msgs \
    --skip-keys dwa_local_planner \
    --skip-keys map_server \
    --skip-keys iginition \
    --skip-keys gazebo_plugins \
    --skip-keys gazebo_ros \
    --skip-keys turtlebot3_navigation2 \
    --skip-keys free_fleet_client_ros1 \
    --skip-keys free_fleet_client_ros2 \
    --skip-keys ff_rviz_plugins_ros1 \
    -y

# Install Node.js 16 and pnpm
RUN curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.37.2/install.sh | bash && \
    export NVM_DIR="$HOME/.nvm" && \
    [ -s "$NVM_DIR/nvm.sh" ] && . "$NVM_DIR/nvm.sh" && \
    nvm install 16 && \
    curl -fsSL https://get.pnpm.io/install.sh | bash - && \
    export PNPM_HOME="/root/.local/share/pnpm" && \
    export PATH="$PNPM_HOME:$PATH" && \
    pnpm env use --global 16 && \
    cd /rmf_ws/src/rmf-web/packages/dashboard && pnpm install

# Set the working directory
WORKDIR /rmf_ws

# Build and clean up
RUN . /opt/ros/iron/setup.sh && \
    colcon build

# Set the default command to bash
CMD ["bash"]

