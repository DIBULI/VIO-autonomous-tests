#!/bin/bash
current_directory="$(cd "$(dirname "$0")" && pwd)"
project_root_dir=${current_directory}/..

mkdir -p ${project_root_dir}/workspace

bash ${current_directory}/repos/dm-vio-init.sh

bash ${current_directory}/repos/vins-fusion-init.sh

bash ${current_directory}/repos/openvins-init.sh