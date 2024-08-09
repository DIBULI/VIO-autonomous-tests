#!/bin/bash
current_directory="$(cd "$(dirname "$0")" && pwd)"
project_root_dir=${current_directory}/..

mkdir -p euroc

BASE_URI=http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/

# https://cvg.cit.tum.de/data/datasets/visual-inertial-dataset
wget -R "index.*" -m -np -nH --no-check-certificate -e robots=off \
    ${BASE_URI}