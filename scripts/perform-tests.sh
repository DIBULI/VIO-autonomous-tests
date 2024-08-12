#!/bin/bash
current_directory="$(cd "$(dirname "$0")" && pwd)"
project_root_dir=${current_directory}/..

DATASET_PATH="${DATASET_PATH:-/data}"

export DATASET_PATH=${DATASET_PATH}

WORKSPACE=${project_root_dir}/workspace

source ${WORKSPACE}/devel/setup.bash

# convert euroc dataset into bags containing ground truth
EUROC_DATASET_PATH=${DATASET_PATH}/euroc
EUROC_SCENRIOS=("machine_hall" "vicon_room1" "vicon_room2")
EUROC_ZIPS=()

for SCENRIOS in "${EUROC_SCENRIOS[@]}"; do
    while IFS= read -r -d '' file; do
        EUROC_ZIPS+=("$file")
    done < <(find "${EUROC_DATASET_PATH}/$SCENRIOS" -name "*.zip" -type f -print0)
done

for ZIP in "${EUROC_ZIPS[@]}"; do
    echo "Found euroc zip file: $ZIP, converting to rosbag"
    SEQUENCE_NAME_WITH_EXT=$(basename "$ZIP")
    SEQUENCE_NAME="${SEQUENCE_NAME_WITH_EXT%.*}"
    DIR_PATH=$(dirname "$ZIP")
    if [ ! -f "$DIR_PATH/${SEQUENCE_NAME}.bag" ]; then
      python3 ${current_directory}/datasets/asl_to_rosbag.py -v --path $ZIP --output $DIR_PATH/${SEQUENCE_NAME}.bag
    fi
done

bash ${current_directory}/tests/dm-vio-tests.sh