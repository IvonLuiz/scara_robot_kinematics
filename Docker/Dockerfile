FROM ubuntu:focal

# Install ROS dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl gnupg2 lsb-release ca-certificates sudo locales tzdata && \
    locale-gen en_US.UTF-8 && \
    ln -fs /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    dpkg-reconfigure --frontend noninteractive tzdata && \
    rm -rf /var/lib/apt/lists/*

ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Add ROS repository and key
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros1.list

# Install ROS Noetic and required packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control \
    ros-noetic-ros-control \
    ros-noetic-ros-controllers \
    build-essential \
    python3-pip \
    git && \
    rm -rf /var/lib/apt/lists/*

# Install python dependencies
RUN pip3 install pandas

# Create workspace
RUN mkdir -p /home/catkin_ws/src
WORKDIR /home/catkin_ws


# Source ROS setup
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "source /home/catkin_ws/devel/setup.bash" >> /root/.bashrc

# Entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
