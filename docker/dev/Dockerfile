FROM osrf/ros:noetic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -q -y --no-install-recommends \
    git terminator python3-pip &&\
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# additional ros dependencies
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential python3-rosdep ros-noetic-catkin\
    ros-noetic-move-base ros-noetic-gmapping && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init || echo "rosdep already initialized" && \
    rosdep update

WORKDIR /

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Useful aliases
RUN echo "alias sr_ros='source /opt/ros/noetic/setup.bash'" >> /root/.bashrc \
    && echo "alias sr='source /root/anscer-robotics-assignment/ws_ros/devel/setup.bash'" >> /root/.bashrc

COPY ros_entrypoint.sh .
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]

WORKDIR /anscer-robotics-assignment/ws_ros/