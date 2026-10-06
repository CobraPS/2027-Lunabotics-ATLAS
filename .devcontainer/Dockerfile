FROM ros:jazzy-ros-base-noble

RUN apt-get update \
    && apt-get install -y --no-install-recommends python3-pip \
    && python3 -m pip install --break-system-packages --no-cache-dir pyntcore==2026.2.2 \
    && rm -rf /var/lib/apt/lists/*

RUN echo 'source /opt/ros/jazzy/setup.bash' >> /home/ubuntu/.bashrc

RUN echo 'ubuntu ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers.d/ubuntu && chmod 0440 /etc/sudoers.d/ubuntu
