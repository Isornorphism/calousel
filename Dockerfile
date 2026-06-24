FROM ros:humble-ros-base-jammy

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV MPLBACKEND=Agg

# Python runtime notes:
# - examples/scripts/evaluate_testbed_results.py uses NumPy and PyYAML.
# - DiscoCal's mono evaluation/uncertainty path imports SciPy, Matplotlib,
#   and OpenCV, so the release image keeps those packages installed.
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    ca-certificates \
    cmake \
    git \
    libatlas-base-dev \
    libeigen3-dev \
    libgflags-dev \
    libgoogle-glog-dev \
    libopencv-dev \
    libsuitesparse-dev \
    libyaml-cpp-dev \
    pybind11-dev \
    python3-colcon-common-extensions \
    python3-matplotlib \
    python3-numpy \
    python3-opencv \
    python3-pip \
    python3-scipy \
    python3-yaml \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-rclcpp \
    ros-${ROS_DISTRO}-rosbag2-cpp \
    ros-${ROS_DISTRO}-rosbag2-storage-default-plugins \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-sophus \
    wget \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /tmp
RUN wget -q http://ceres-solver.org/ceres-solver-2.2.0.tar.gz && \
    tar xf ceres-solver-2.2.0.tar.gz && \
    cmake -S ceres-solver-2.2.0 -B ceres-build \
      -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_EXAMPLES=OFF \
      -DBUILD_TESTING=OFF \
      -DMINIGLOG=OFF && \
    cmake --build ceres-build -j"$(nproc)" && \
    cmake --install ceres-build && \
    rm -rf /tmp/ceres-solver-2.2.0 /tmp/ceres-solver-2.2.0.tar.gz /tmp/ceres-build

WORKDIR /workspace
COPY . /workspace

RUN test -f src/discocal/CMakeLists.txt || \
    (echo "src/discocal is missing. Run: git submodule update --init --recursive" && false)

RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build \
      --merge-install \
      --packages-select discocal calousel camera_image_saver \
      --cmake-args -DCMAKE_BUILD_TYPE=Release

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
