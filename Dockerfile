FROM ros:noetic-robot-focal

ARG WORKDIR=/root
WORKDIR ${WORKDIR}

# first install several commonly used dependency

RUN apt-get update && apt-get install -y \
    vim \
    python3-pip \
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
    # required by pangolin
    libgl1-mesa-dev libglew-dev pkg-config libegl1-mesa-dev libwayland-dev libxkbcommon-dev wayland-protocols \
    ros-noetic-catkin python3-catkin-tools ros-noetic-cv-bridge \
    # required by vins-fusion
    ros-noetic-image-transport \
    # required by openvins
    libceres-dev && \ 
    apt-get clean autoclean && \
    apt-get autoremove --yes && \
    rm -rf /var/lib/apt/lists/* && \
    rm -rf /var/lib/{apt,dpkg,cache,log}/

RUN pip3 install \
    # required by dm-vio-python-tools
    tqdm ruamel.yaml pyquaternion matplotlib tabulate \
    numpy==1.24.4 evo==1.29.0 rosbag-merge \
    # required by converting euroc dataset
    argparse pandas pyyaml

COPY scripts/repos ${WORKDIR}/VIO-auto-tests/scripts/repos
COPY scripts/init-workspace.sh ${WORKDIR}/VIO-auto-tests/scripts/init-workspace.sh

RUN bash ${WORKDIR}/VIO-auto-tests/scripts/init-workspace.sh

COPY scripts/build-workspace.sh ${WORKDIR}/VIO-auto-tests/scripts/build-workspace.sh
RUN bash ${WORKDIR}/VIO-auto-tests/scripts/build-workspace.sh

COPY scripts/datasets ${WORKDIR}/VIO-auto-tests/scripts/datasets
COPY scripts/tests ${WORKDIR}/VIO-auto-tests/scripts/tests
COPY scripts/perform-tests.sh ${WORKDIR}/VIO-auto-tests/scripts/perform-tests.sh

ENTRYPOINT ["/bin/bash", "-i", "/root/VIO-auto-tests/scripts/perform-tests.sh"]
