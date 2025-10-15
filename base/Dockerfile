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
#RUN pip3 install --break-system-packages matplotlib transforms3d
RUN pip3 install $([ "${ROS_DISTRO}" = "jazzy" ] && echo "--break-system-packages" || echo "") matplotlib transforms3d

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
#RUN pip install --break-system-packages setuptools==58.2.0
RUN pip3 install $([ "${ROS_DISTRO}" = "jazzy" ] && echo "--break-system-packages" || echo "") setuptools==58.2.0

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
# Fix empy version for Humble (requires empy<4.0)
RUN if [ "${ROS_DISTRO}" = "humble" ]; then pip install 'empy<4.0'; fi

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

# Source ROS environment in /etc/bash.bashrc for docker exec sessions
# This ensures ROS is available when using `docker exec -it <container> bash`
RUN echo "# Auto-source ROS 2 environment" >> /etc/bash.bashrc && \
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/bash.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> /etc/bash.bashrc && \
    echo "# Source dev workspaces if they exist" >> /etc/bash.bashrc && \
    echo "if [ -f ~/dev_ws/install/setup.bash ]; then source ~/dev_ws/install/setup.bash; fi" >> /etc/bash.bashrc && \
    echo "if [ -f ~/sim_ws/install/setup.bash ]; then source ~/sim_ws/install/setup.bash; fi" >> /etc/bash.bashrc && \
    echo "# Suppress daemon timeout warnings (known issue in Docker)" >> /etc/bash.bashrc && \
    echo "export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET" >> /etc/bash.bashrc

# Create dev user for interactive development
# - UID 1000 matches typical WSL2 user for bind-mount permissions
# - Remove default ubuntu user if it exists (to avoid UID conflict)
# - Add to sudo and video groups for GUI support
# Note: fuse group not needed - AppImages work without it on recent systems
RUN userdel -r ubuntu 2>/dev/null || true && \
    groupadd -g 1000 dev && \
    useradd -m -u 1000 -g 1000 -G sudo,video -s /bin/bash dev && \
    echo "dev:dev" | chpasswd && \
    echo "dev ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Create dev workspace directories with proper permissions
RUN mkdir -p /home/dev/dev_ws /home/dev/sim_ws && \
    chown -R dev:dev /home/dev

# Copy Groot2 to dev user home (was in /root)
RUN cp /root/Groot2.AppImage /home/dev/ && \
    chown dev:dev /home/dev/Groot2.AppImage

# Add entrypoint
WORKDIR /ros2_ws
COPY ./base/entrypoint.sh /
# Fix potential Windows line ending issues and make executable
RUN sed -i 's/\r$//' /entrypoint.sh && chmod +x /entrypoint.sh
ENTRYPOINT [ "/bin/bash", "/entrypoint.sh" ]

# Default to dev user for interactive sessions
# Compose files can override with user: root if needed
USER dev
WORKDIR /home/dev

####################################
# Dev Stage - Additional Dev Tools #
####################################
FROM base AS dev

# Switch back to root for installing packages
USER root

# Install MoveIt2 - required for roarm_description and manipulation development
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-moveit \
    && rm -rf /var/lib/apt/lists/*

# Install Navigation2 (Nav2) - autonomous navigation stack
# Provides path planning, behavior trees, collision avoidance, and map management
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    && rm -rf /var/lib/apt/lists/*

# Install Intel RealSense SDK (userspace libraries only)
# Note: Skip librealsense2-dkms (kernel module) - not needed in containers
# The host system should have the kernel module if using real hardware
RUN apt-get update && apt-get install -y --no-install-recommends \
    software-properties-common \
    && apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || \
    apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE \
    && add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u \
    && apt-get install -y --no-install-recommends \
    librealsense2-utils \
    librealsense2-dev \
    && rm -rf /var/lib/apt/lists/*

# Install RealSense ROS wrapper (if available in ROS repos)
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-realsense2-camera \
    ros-${ROS_DISTRO}-realsense2-description \
    || echo "RealSense ROS packages not available, will build from source" \
    && rm -rf /var/lib/apt/lists/*

# Install ROS 2 Control packages (for moveit_servo and manipulation)
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-control-toolbox \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-joint-state-broadcaster \
    ros-${ROS_DISTRO}-joint-trajectory-controller \
    ros-${ROS_DISTRO}-gripper-controllers \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    && rm -rf /var/lib/apt/lists/*

# Install audio_common packages (for by_your_command voice control)
# Note: audio_common not in Humble repos, will be built from source
# Install its dependencies: PortAudio, ALSA
RUN apt-get update && apt-get install -y --no-install-recommends \
    portaudio19-dev \
    libasound2-dev \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-ugly \
    && rm -rf /var/lib/apt/lists/*

# Install additional MoveIt2 dependencies (optional, may not be available in all distros)
RUN apt-get update && \
    (apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-moveit-resources-panda-moveit-config \
        ros-${ROS_DISTRO}-moveit-visual-tools \
        ros-${ROS_DISTRO}-launch-pytest \
        python3-tqdm \
        python3-requests \
        || echo "Some optional MoveIt2 packages not available") && \
    rm -rf /var/lib/apt/lists/*

# Switch back to dev user
USER dev
WORKDIR /home/dev
