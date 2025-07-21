# ==========================
# Dockerfile: ROS1 Noetic + ROS2 Foxy + ros1_bridge + rosbag2_storage_rosbag_v2
# ==========================

FROM osrf/ros:foxy-desktop-focal

ENV DEBIAN_FRONTEND=noninteractive

# Enable universe & multiverse for missing dependencies
RUN apt-get update && \
    apt-get install -y software-properties-common && \
    add-apt-repository universe && \
    add-apt-repository multiverse && \
    apt-get update

# Install Python 3.8 explicitly
RUN apt-get install -y python3.8 python3.8-dev python3.8-distutils

# Install ROS 1 Noetic sources and keys
RUN apt-get install -y \
    curl gnupg2 lsb-release && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | \
    gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu focal main" \
    > /etc/apt/sources.list.d/ros1.list && \
    apt-get update

# Install ROS 1 and build tools
RUN apt-get install -y \
    ros-noetic-desktop-full \
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-rosinstall-generator \
    build-essential \
    git \
    cmake

# Initialize rosdep
RUN if [ -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
    rm /etc/ros/rosdep/sources.list.d/20-default.list; \
    fi && \
    rosdep init && rosdep update

# Create workspace
WORKDIR /root/ros_ws
RUN mkdir -p src
WORKDIR /root/ros_ws/src

# Clone necessary packages
RUN git clone -b foxy https://github.com/ros2/rosbag2.git && \
    git clone -b foxy https://github.com/ros2/ros1_bridge.git && \
    git clone -b foxy https://github.com/ros2/common_interfaces.git

# Install dependencies via rosdep
WORKDIR /root/ros_ws
RUN . /opt/ros/noetic/setup.sh && \
    . /opt/ros/foxy/setup.sh && \
    rosdep install --from-paths src --ignore-src -r -y

# Build workspace
RUN . /opt/ros/noetic/setup.sh && \
    . /opt/ros/foxy/setup.sh && \
    colcon build --packages-select rosbag2_storage_rosbag_v2 ros1_bridge --cmake-args -DCMAKE_BUILD_TYPE=Release

# Add entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
