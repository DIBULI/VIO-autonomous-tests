#!/bin/bash
current_directory="$(cd "$(dirname "$0")" && pwd)"
project_root_dir=${current_directory}/../..

# install gtsam
mkdir -p ${current_directory}/third_party && cd ${current_directory}/third_party
git clone https://github.com/borglab/gtsam.git
cd gtsam
git checkout a738529af9754c7a085903f90ae8559bbaa82e75          # newer gtsam versions might not work.
mkdir build && cd build
cmake -DGTSAM_POSE3_EXPMAP=ON -DGTSAM_ROT3_EXPMAP=ON -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF ..
make -j 8
sudo make install

cd ${current_directory}/third_party
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
git checkout v0.6
mkdir build
cd build
cmake ..
cmake --build . -j 8
sudo make install

# remove third_party
rm -rf ${current_directory}/third_party

mkdir -p ${project_root_dir}/workspace/src && cd ${project_root_dir}/workspace/src
# build dm-vio
git clone https://github.com/lukasvst/dm-vio.git
git checkout d18fa15ba086043561361941b8e5298074b34b47
cd dm-vio && touch CATKIN_IGNORE
mkdir build
cd build
cmake ..
make -j 8


# build the dm-vio-ros
echo "export DMVIO_BUILD=${project_root_dir}/workspace/src/dm-vio/build" >> ~/.bashrc
cd ${project_root_dir}/workspace/src
git clone https://github.com/lukasvst/dm-vio-ros.git

cp /usr/local/lib/libmetis.so /usr/lib