#!/bin/bash
current_directory="$(cd "$(dirname "$0")" && pwd)"
project_root_dir=${current_directory}/../..

mkdir -p ${project_root_dir}/workspace/src && cd ${project_root_dir}/workspace

cd ${current_directory}/third_party
mkdir -p ceres_solver && cd ceres_solver
git clone https://ceres-solver.googlesource.com/ceres-solver
cd ceres-solver
git checkout 1.14.0
mkdir build && cd build
cmake ..
make -j8
sudo make install

cd ${project_root_dir}/workspace/src
git clone https://github.com/HalfVulpes/vins-fusion-cv4.git

echo "VINS-Fusion initialization completed."
