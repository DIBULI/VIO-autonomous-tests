#!/bin/bash
current_directory="$(cd "$(dirname "$0")" && pwd)"
project_root_dir=${current_directory}/..

DATASET_PATH="${DATASET_PATH:-/data}"

export DATASET_PATH=${DATASET_PATH}

WORKSPACE=${project_root_dir}/workspace

source ${WORKSPACE}/devel/setup.bash

bash ${current_directory}/tests/dm-vio-tests.sh