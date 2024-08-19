#!/bin/bash
current_directory="$(cd "$(dirname "$0")" && pwd)"
project_root_dir=${current_directory}/..

DATASET_PATH="${DATASET_PATH:-/data}"
TEST_RESULT_PATH="${TEST_RESULT_PATH:-/root/VIO-auto-tests/test-results}"

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

# for each euroc bag, run the following result
EUROC_DATASET_PATH=${DATASET_PATH}/euroc
EUROC_SCENRIOS=("machine_hall" "vicon_room1" "vicon_room2")

EUROC_BAGS=()

for SCENRIOS in "${EUROC_SCENRIOS[@]}"; do
    while IFS= read -r -d '' file; do
        EUROC_BAGS+=("$file")
    done < <(find "${EUROC_DATASET_PATH}/$SCENRIOS" -name "*.bag" -type f -print0)
done


echo "Found bags:"
for BAG in "${EUROC_BAGS[@]}"; do
    echo "$BAG"
done

mkdir -p $TEST_RESULT_PATH && cd $TEST_RESULT_PATH

# dm-vio tests
rm -rf dm-vio-tests
mkdir -p dm-vio-tests
cd dm-vio-tests

# euroc tests
mkdir -p euroc
for BAG in "${EUROC_BAGS[@]}"; do
  echo "-------------------------- processing dataset: ${BAG} ------------------------------"
  SEQUENCE_NAME_WITH_EXT=$(basename "$BAG")
  SEQUENCE_NAME="${SEQUENCE_NAME_WITH_EXT%.*}"

  mkdir -p $TEST_RESULT_PATH/dm-vio-tests/euroc/$SEQUENCE_NAME

  bash $current_directory/tests/dm-vio-tests.sh $TEST_RESULT_PATH/dm-vio-tests/euroc/$SEQUENCE_NAME $SEQUENCE_NAME $BAG /state_groundtruth_estimate0 
done
cd -

# vins-fusion tests
rm -rf vins-fusion-tests
mkdir -p vins-fusion-tests
cd vins-fusion-tests

mkdir -p euroc
for BAG in "${EUROC_BAGS[@]}"; do
    echo "-------------------------- processing dataset: ${BAG} ------------------------------"
    SEQUENCE_NAME_WITH_EXT=$(basename "$BAG")
    SEQUENCE_NAME="${SEQUENCE_NAME_WITH_EXT%.*}"

    mkdir -p $TEST_RESULT_PATH/vins-fusion-tests/euroc/$SEQUENCE_NAME
    
    bash ${current_directory}/tests/vins-fusion-tests.sh $TEST_RESULT_PATH/vins-fusion-tests/euroc/$SEQUENCE_NAME $SEQUENCE_NAME $BAG /state_groundtruth_estimate0 \
        ${project_root_dir}/workspace/src/vins-fusion-cv4/config/euroc/euroc_stereo_imu_config.yaml
done
cd -