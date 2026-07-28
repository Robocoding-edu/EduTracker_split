FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    libudev-dev \
    python3-serial \
    python3-pip \
    python3-numpy \
    ros-jazzy-cv-bridge \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /data

CMD ["bash", "-c", "source /opt/ros/jazzy/setup.bash && \
if [ -f /data/src/install/setup.bash ]; then \
    source /data/src/install/setup.bash; \
fi && \
./start.sh"]
