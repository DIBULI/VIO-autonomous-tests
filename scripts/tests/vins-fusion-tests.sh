#!/bin/bash

current_directory="$(cd "$(dirname "$0")" && pwd)"
project_root_dir=${current_directory}/../..

DATASET_PATH=$DATASET_PATH
TEST_RESULT_PATH=${project_root_dir}/tests_result

mkdir -p ${TEST_RESULT_PATH} && cd ${TEST_RESULT_PATH}

rm -rf vins-fusion-tests
mkdir -p vins-fusion-tests
cd vins-fusion-tests

roscore &
ROSCORE_PID=$!

sleep 2

EUROC_DATASET_PATH=${DATASET_PATH}/euroc
EUROC_SCENARIOS=("machine_hall" "vicon_room1" "vicon_room2")

EUROC_BAGS=()

for SCENARIO in "${EUROC_SCENARIOS[@]}"; do
    while IFS= read -r -d '' file; do
        EUROC_BAGS+=("$file")
    done < <(find "${EUROC_DATASET_PATH}/$SCENARIO" -name "*.bag" -type f -print0)
done

echo "Found bags:"
for BAG in "${EUROC_BAGS[@]}"; do
    echo "$BAG"
done

mkdir -p euroc
for BAG in "${EUROC_BAGS[@]}"; do
  echo "-------------------------- processing dataset: ${BAG} ------------------------------"
  SEQUENCE_NAME_WITH_EXT=$(basename "$BAG")
  SEQUENCE_NAME="${SEQUENCE_NAME_WITH_EXT%.*}"

  mkdir -p euroc/${SEQUENCE_NAME}
  cd euroc/${SEQUENCE_NAME}
  
  echo -e "458.654 457.296 367.215 248.375 -0.28340811 0.07395907 0.00019359 1.76187114e-05\n752 480\ncrop\n640 480\n" > camera.txt

  rosrun vins vins_node ${project_root_dir}/workspace/src/vins-fusion-cv4/config/euroc/euroc_stereo_imu_config.yaml \
    > ${SEQUENCE_NAME}.output &
  VINS_FUSION_PID=$!

  sleep 5

  echo "Start to record the odometry result"
  rosbag record -O odom_result /vins_estimator/odometry > rosbag.output 2>&1 &
  ROSBAG_RECORD_PID=$!
  sleep 1

  echo "Start to play the dataset bag"
  rosbag play ${BAG} &
  ROSBAG_PLAY_PID=$!

  CSV_FILE=system.csv
  touch ${CSV_FILE}

  while ps -p $ROSBAG_PLAY_PID > /dev/null 2>&1; do
    TIMESTAMP=$(date +%s%3N)
    CPU_MEM_USAGE=$(ps -p $VINS_FUSION_PID -o %cpu,%mem --no-headers)

    if [ -n "$CPU_MEM_USAGE" ]; then
        echo "$TIMESTAMP $CPU_MEM_USAGE" >> "$CSV_FILE"
    fi

    sleep 0.1
  done

  sleep 5
  echo "Killing rosbag recording process"
  while kill -SIGINT $ROSBAG_RECORD_PID 2>/dev/null; do
    echo "Waiting for process $ROSBAG_RECORD_PID to be killed..."
    sleep 1
  done
  echo "Killing VINS-Fusion process"
  kill $VINS_FUSION_PID

  python3 ${current_directory}/python/cal_rosbag_frequency.py odom_result.bag /vins_estimator/odometry > odom_result.hz

  ln -s ${BAG} ${SEQUENCE_NAME}.bag
  rosbag-merge --outbag_name result --topics /vins_estimator/odometry /state_groundtruth_estimate0  --write_bag
  evo_traj bag result.bag /vins_estimator/odometry --save_plot plot.pdf --full_check -as --ref /state_groundtruth_estimate0 
  evo_ape bag result.bag /state_groundtruth_estimate0  /vins_estimator/odometry -as
  evo_rpe bag result.bag /state_groundtruth_estimate0 /vins_estimator/odometry -as

  cd -
done

# Kill
kill $ROSCORE_PID
