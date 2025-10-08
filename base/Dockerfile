ARG ROS_DISTRO=jazzy
ARG GZ_VERSION=gz-harmonic
####################################
# Base Image for Grunt Type Robots #
####################################
FROM osrf/ros:${ROS_DISTRO}-desktop AS base

# re-declare args as a keep-alive after "FROM"
ARG ROS_DISTRO
ARG GZ_VERSION

ENV ROS_DISTRO=${ROS_DISTRO}
ENV GZ_VERSION=${GZ_VERSION}

# Install basic apt packages
RUN apt-get update && apt-get install -y --no-install-recommends \
 curl git wget libcanberra-gtk-module libcanberra-gtk3-module fuse3 libfuse2 libqt5svg5-dev \
 python3-pip python3-opencv python3-tk python3-pyqt5.qtwebengine \
 iputils-ping net-tools iproute2 nano vim gedit

# Install additional Python modules
RUN pip3 install --break-system-packages matplotlib transforms3d

# Include Cyclone DDS as alternate middleware
RUN apt-get update && apt-get install -y --no-install-recommends \
 ros-${ROS_DISTRO}-rmw-cyclonedds-cpp

# Choose RMW. Since this an environment variable, it can be changed at runtime (see bashrc_custom), 
# but needs to be the same for all particpating systems
# ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Install GZ Sim (binary install: https://gazebosim.org/docs/garden/install_ubuntu)
# Specific version selected with GZ_VERSION arg
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Update and install packages separately for better debugging
RUN apt update -y

# Install Gazebo vendor packages for ROS2 Jazzy
# These are the recommended packages for ROS2 Jazzy integration with Gazebo Harmonic
RUN apt install -y \
    ros-${ROS_DISTRO}-gz-cmake-vendor \
    ros-${ROS_DISTRO}-gz-sim-vendor \
    ros-${ROS_DISTRO}-ros-gz-interfaces \
    ros-${ROS_DISTRO}-ros-gz-bridge || echo "Vendor packages not found, trying alternatives..."

# Try the meta-package as fallback
RUN apt install -y ros-${ROS_DISTRO}-ros-gz || echo "Meta-package not found, trying standalone..."

# Try standalone Gazebo as last resort
RUN apt install -y gz-harmonic || echo "Standalone package not found, continuing anyway..."

# Install other dependencies
RUN apt install -y libgflags-dev \
    ros-${ROS_DISTRO}-gps-msgs \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-actuator-msgs \
    ros-${ROS_DISTRO}-vision-msgs \
    python3-pip
RUN pip install --break-system-packages setuptools==58.2.0

# Clean up apt package list
RUN rm -rf /var/lib/apt/lists/*

# Create Colcon workspace with external dependencies
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws/src

# Create a Python virtual environment for tools
RUN apt-get update && apt-get install -y python3-venv
RUN python3 -m venv /opt/venv
ENV PATH="/opt/venv/bin:$PATH"

# Install tools in the virtual environment
RUN pip install vcstool rosdep colcon-common-extensions
RUN pip install numpy lark

# Initialize rosdep
RUN rosdep init || echo "rosdep already initialized"
RUN rosdep update

COPY dependencies.repos .
# Use the vcs from the virtual environment
RUN /opt/venv/bin/vcs import < dependencies.repos

# temporary - Create missing directories that might be referenced in CMakeLists.txt
#RUN mkdir -p /ros2_ws/src/grunt/grunt_description/launch
#RUN mkdir -p /ros2_ws/src/grunt/grunt_util/config

# Build the base Colcon workspace, installing dependencies first.
WORKDIR /ros2_ws
RUN bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash \
  && apt-get update -y \
  && /opt/venv/bin/rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y --skip-keys='rviz' \
  && /opt/venv/bin/colcon build --symlink-install"

# Download Groot2 AppImage and place it in the home folder.
WORKDIR /root/
RUN curl -o Groot2.AppImage https://s3.us-west-1.amazonaws.com/download.behaviortree.dev/groot2_linux_installer/Groot2-v1.6.1-x86_64.AppImage \
 && chmod a+x Groot2.AppImage

# Remove display warnings - these are useful for X11 - 
# not needed if using Wayland on WSL2
# RUN mkdir /tmp/runtime-root
# ENV XDG_RUNTIME_DIR "/tmp/runtime-root"
# RUN chmod -R 0700 /tmp/runtime-root
# ENV NO_AT_BRIDGE 1

# Add entrypoint
WORKDIR /ros2_ws
COPY ./base/entrypoint.sh /
# Fix potential Windows line ending issues and make executable
RUN sed -i 's/\r$//' /entrypoint.sh && chmod +x /entrypoint.sh
ENTRYPOINT [ "/bin/bash", "/entrypoint.sh" ]
