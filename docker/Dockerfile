FROM ros:kinetic-perception

ENV CERES_VERSION="1.12.0"
ENV CATKIN_WS=/root/catkin_ws

RUN   apt-get update && apt-get install -y \
      cmake \
      libatlas-base-dev \
      libeigen3-dev \
      libgoogle-glog-dev \
      libsuitesparse-dev \
      libatlas-base-dev \
      python-catkin-tools \
      libboost-dev libboost-filesystem-dev \
      libopencv-dev \
      ros-${ROS_DISTRO}-cv-bridge \
      ros-${ROS_DISTRO}-image-transport \
      ros-${ROS_DISTRO}-message-filters \
      ros-${ROS_DISTRO}-tf && \
      ros-${ROS_DISTRO}-rviz && \
      ros-${ROS_DISTRO}-pcl-ros \
      rm -rf /var/lib/apt/lists/* && \
      mkdir -p $CATKIN_WS/src/OKVIS/

# Copy OKVIS
COPY ./ $CATKIN_WS/src/OKVIS/

# Build OKVIS
WORKDIR $CATKIN_WS
ENV TERM xterm
ENV PYTHONIOENCODING UTF-8
RUN catkin config \
      --extend /opt/ros/$ROS_DISTRO \
      --cmake-args \
        -DCMAKE_BUILD_TYPE=Release && \
    catkin build && \
    sed -i '/exec "$@"/i \
            source "/root/catkin_ws/devel/setup.bash"' /ros_entrypoint.sh
