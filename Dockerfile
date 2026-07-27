FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    libudev-dev \
    python3-serial \
    python3-pip \
    python3-numpy \
    ros-jazzy-slam-toolbox \
    ros-jazzy-cv-bridge \
    && rm -rf /var/lib/apt/lists/*


CMD ["bash", "-c", "source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && ./start_all.sh"]
