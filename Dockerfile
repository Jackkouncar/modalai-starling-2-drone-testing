# syntax=docker/dockerfile:1
FROM ubuntu:22.04

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ARG ROS_DISTRO=humble
ARG PX4_REPO=https://github.com/PX4/PX4-Autopilot.git
ARG PX4_REF=release/1.14
ARG PX4_MSGS_REPO=https://github.com/PX4/px4_msgs.git
ARG PX4_MSGS_REF=release/1.14
ARG MICRO_XRCE_DDS_AGENT_REF=v2.4.3

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8 \
    ROS_DISTRO=${ROS_DISTRO} \
    ROS_WS=/workspace/ros2_ws \
    PX4_HOME=/workspace/PX4-Autopilot \
    DRONE_SIM=1 \
    PX4_SIM_TARGET=gz_x500_depth \
    GZ_VERSION=garden \
    PIP_DEFAULT_TIMEOUT=120 \
    PIP_RETRIES=10 \
    PIP_DISABLE_PIP_VERSION_CHECK=1 \
    PIP_PROGRESS_BAR=off \
    PATH=/root/.local/bin:${PATH}

RUN apt-get update \
    && apt-get install -y --no-install-recommends software-properties-common \
    && add-apt-repository universe \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        bash-completion \
        build-essential \
        ca-certificates \
        ccache \
        cmake \
        curl \
        dbus-x11 \
        git \
        gnupg2 \
        lsb-release \
        libasio-dev \
        libgl1-mesa-dri \
        libglx-mesa0 \
        libssl-dev \
        libtinyxml2-dev \
        locales \
        mesa-utils \
        ninja-build \
        pkg-config \
        python3-dev \
        python3-pip \
        python3-setuptools \
        python3-venv \
        python3-wheel \
        sudo \
        unzip \
        wget \
        x11-apps \
        xz-utils \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "${UBUNTU_CODENAME}") main" \
        > /etc/apt/sources.list.d/ros2.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-rmw-fastrtps-cpp \
        ros-${ROS_DISTRO}-ros-base \
        ros-dev-tools \
    && (rosdep init || true) \
    && rosdep update --rosdistro ${ROS_DISTRO} \
    && rm -rf /var/lib/apt/lists/*

RUN git clone --branch "${PX4_REF}" --depth 1 "${PX4_REPO}" "${PX4_HOME}" \
    && cd "${PX4_HOME}" \
    && git submodule update --init --recursive --depth 1 \
    && bash ./Tools/setup/ubuntu.sh --no-nuttx \
    && (python3 -m pip uninstall -y em || true) \
    && python3 -m pip install --no-cache-dir empy==3.3.4

RUN apt-get update \
    && apt-get install -y --no-install-recommends gz-harmonic libunwind-dev \
    && rm -rf /var/lib/apt/lists/*

RUN git clone --branch "${MICRO_XRCE_DDS_AGENT_REF}" --depth 1 \
        https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/Micro-XRCE-DDS-Agent \
    && cd /tmp/Micro-XRCE-DDS-Agent \
    && git submodule update --init --recursive \
    && mkdir build \
    && cd build \
    && cmake .. \
    && cmake --build . --parallel "$(nproc)" \
    && cmake --install . \
    && ldconfig /usr/local/lib \
    && rm -rf /tmp/Micro-XRCE-DDS-Agent

RUN mkdir -p "${ROS_WS}/src" \
    && git clone --branch "${PX4_MSGS_REF}" --depth 1 "${PX4_MSGS_REPO}" "${ROS_WS}/src/px4_msgs" \
    && source "/opt/ros/${ROS_DISTRO}/setup.bash" \
    && cd "${ROS_WS}" \
    && colcon build --symlink-install

COPY docker/entrypoint.sh /usr/local/bin/drone-sim-entrypoint
RUN chmod +x /usr/local/bin/drone-sim-entrypoint

WORKDIR /workspace/drone-tests
COPY . /workspace/drone-tests
RUN chmod +x /workspace/drone-tests/docker/*.sh

ENTRYPOINT ["/usr/local/bin/drone-sim-entrypoint"]
CMD ["bash"]
