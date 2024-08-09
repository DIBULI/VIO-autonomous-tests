#!/bin/bash
current_directory="$(cd "$(dirname "$0")" && pwd)"
project_root_dir=${current_directory}/..

DATA_PATH=~/data

mkdir -p ${DATA_PATH}/dataset/vios

bash -c "cd ${DATA_PATH}/dataset/vios && bash ${current_directory}/tumvi-dataset.sh"

bash -c "cd ${DATA_PATH}/dataset/vios && bash ${current_directory}/euroc-dataset.sh"