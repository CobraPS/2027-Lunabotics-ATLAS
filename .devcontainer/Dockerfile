FROM ros:jazzy-ros-base-noble

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        python3-pip \
        libpcl-dev \
        ros-jazzy-pcl-conversions \
        ros-jazzy-tf2-ros \
        ros-jazzy-rviz2 \
        python3-colcon-common-extensions \
        python3-pytest \
        python3-numpy \
        python3-yaml \
        libgl1-mesa-dri \
        libegl1 \
        xauth \
        x11-utils \
        mesa-utils \
        qtwayland5 \
        qt6-wayland \
        ros-jazzy-ros-gz \
        ros-jazzy-xacro \
        ros-jazzy-sensor-msgs-py \
        ros-jazzy-ament-copyright \
        ros-jazzy-ament-flake8 \
        ros-jazzy-ament-pep257 \
        ros-jazzy-navigation2 \
        ros-jazzy-nav2-bringup \
        ros-jazzy-slam-toolbox \
    && python3 -m pip install --break-system-packages --no-cache-dir pyntcore==2026.2.2 \
    && rm -rf /var/lib/apt/lists/*

# Ensure XDG_RUNTIME_DIR exists for Wayland socket bind-mount.
# postCreateCommand in gui-wayland/devcontainer.json re-asserts owner and mode 0700
# after updateRemoteUserUID may have changed the container user's UID.
RUN install -d -m 0700 -o ubuntu -g ubuntu /run/user/1000

RUN echo 'source /opt/ros/jazzy/setup.bash' >> /home/ubuntu/.bashrc

RUN echo 'ubuntu ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers.d/ubuntu && chmod 0440 /etc/sudoers.d/ubuntu
