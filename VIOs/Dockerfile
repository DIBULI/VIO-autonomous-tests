FROM ros:noetic-robot-focal

ARG WORKDIR=/root
WORKDIR ${WORKDIR}

# first install several commonly used dependency

RUN apt-get update && apt-get install -y \
    libsuitesparse-dev \
    libpcap-dev \
    python-is-python3 \
    git \
    cmake \
    build-essential \
    libsuitesparse-dev \
    libeigen3-dev \
    libboost-all-dev \
    libyaml-cpp-dev \
    libtbb-dev \
    libopencv-dev \
    # pangolin required
    libgl1-mesa-dev libglew-dev pkg-config libegl1-mesa-dev libwayland-dev libxkbcommon-dev wayland-protocols&& \ 
    apt-get clean autoclean && \
    apt-get autoremove --yes && \
    rm -rf /var/lib/apt/lists/* && \
    rm -rf /var/lib/{apt,dpkg,cache,log}/

COPY . ${WORKDIR}/VIO-auto-tests

RUN bash -c "bash ${WORKDIR}/VIO-auto-tests/scripts/init-workspace.sh"