ARG ROS_DISTRO=jazzy
ARG GZ_VERSION=gz-harmonic
####################################
# Base Image for Grunt Type Robots #
####################################
FROM osrf/ros:${ROS_DISTRO}-desktop as base
ENV ROS_DISTRO=${ROS_DISTRO}
ENV GZ_VERSION=${GZ_VERSION}

# Install basic apt packages
RUN apt-get update && apt-get install -y --no-install-recommends \
 curl git libcanberra-gtk-module libcanberra-gtk3-module fuse3 libfuse2 libqt5svg5-dev \
 python3-pip python3-opencv python3-tk python3-pyqt5.qtwebengine

# Install additional Python modules
RUN pip3 install --break-system-packages matplotlib transforms3d

# Include Cyclone DDS as alternate middleware
RUN apt-get update && apt-get install -y --no-install-recommends \
 ros-${ROS_DISTRO}-rmw-cyclonedds-cpp

# Choose RMW. Since this an environment variable, it can be changed at runtime (see bashrc_custom), 
# but needs to be the same for all particpating systems
# ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Install Gazebo Garden (binary install: https://gazebosim.org/docs/garden/install_ubuntu)
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN apt update && \
    apt install -y ${GZ_VERSION} \
    ros-cmake-modules \
    libgflags-dev \
    ros-${ROS_DISTRO}-gps-msgs \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-actuator-msgs \
    ros-${ROS_DISTRO}-vision-msgs \
    python3-pip && \
    pip install setuptools==58.2.0

# Clean up apt package list
RUN rm -rf /var/lib/apt/lists/*

# Create Colcon workspace with external dependencies
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws/src
COPY dependencies.repos .
RUN vcs import < dependencies.repos