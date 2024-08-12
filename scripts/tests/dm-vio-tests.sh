#!/bin/bash
current_directory="$(cd "$(dirname "$0")" && pwd)"
project_root_dir=${current_directory}/../..


DATASET_PATH=$DATASET_PATH
TEST_RESULT_PATH=${project_root_dir}/tests_result

mkdir -p ${TEST_RESULT_PATH} && cd ${TEST_RESULT_PATH}

rm -rf dm-vio-tests
mkdir -p dm-vio-tests
cd dm-vio-tests

# start the roscore
roscore &
ROSCORE_PID=$!

sleep 2

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

mkdir -p euroc
for BAG in "${EUROC_BAGS[@]}"; do
  echo "-------------------------- processing dataset: ${BAG} ------------------------------"
  SEQUENCE_NAME_WITH_EXT=$(basename "$BAG")
  SEQUENCE_NAME="${SEQUENCE_NAME_WITH_EXT%.*}"

  mkdir -p euroc/${SEQUENCE_NAME}
  cd euroc/${SEQUENCE_NAME}
  
  # camera intrinsic
  echo -e "458.654 457.296 367.215 248.375 -0.28340811 0.07395907 0.00019359 1.76187114e-05\n752 480\ncrop\n640 480\n" > camera.txt

  rosrun dmvio_ros node calib=${TEST_RESULT_PATH}/dm-vio-tests/euroc/${SEQUENCE_NAME}/camera.txt \
    settingsFile=$DMVIO_BUILD/../configs/euroc.yaml \
    mode=1 nogui=1 preset=1 useimu=1 quiet=0 init_requestFullResetNormalizedErrorThreshold=0.8 init_pgba_skipFirstKFs=1 \
    > ${SEQUENCE_NAME}.output &
  DMVIO_ROS_PID=$!

  # wait for dmvio_ros to be initialized completely
  sleep 5

  # for each result, we save the odometry result as a ros bag
  echo "Start to record the odometry result"
  # use output redirect function to avoid seeing the error message when try to kill the rosbag process
  # https://github.com/ros/ros_comm/issues/2235
  rosbag record -O odom_result /dmvio/unscaled_pose > rosbag.output 2>&1 &
  ROSBAG_RECORD_PID=$!
  sleep 1

  echo "Start to play the dataset bag"
  rosbag play ${BAG} &
  ROSBAG_PLAY_PID=$!

  CSV_FILE=system.csv
  touch ${CSV_FILE}

  while ps -p $ROSBAG_PLAY_PID > /dev/null 2>&1; do
    TIMESTAMP=$(date +%s%3N)
    CPU_MEM_USAGE=$(ps -p $DMVIO_ROS_PID -o %cpu,%mem --no-headers)

    if [ -n "$CPU_MEM_USAGE" ]; then
        echo "$TIMESTAMP $CPU_MEM_USAGE" >> "$CSV_FILE"
    fi

    sleep 0.1
  done

  sleep 5
  echo "Killing rosbag process"
  while kill -SIGINT $ROSBAG_RECORD_PID 2>/dev/null; do
    echo "Waiting for process $ROSBAG_RECORD_PID to be killed..."
    sleep 1
  done
  echo "Killing dmvio ros process"
  kill $DMVIO_ROS_PID

  python3 ${current_directory}/python/cal_rosbag_frequency.py odom_result.bag /dmvio/unscaled_pose > odom_result.hz

  # merge the bags of result and grount truth into one bag
  ln -s ${BAG} ${SEQUENCE_NAME}.bag
  rosbag-merge --outbag_name result --topics /state_groundtruth_estimate0 /dmvio/unscaled_pose --write_bag
  evo_traj bag result.bag /dmvio/unscaled_pose --save_plot plot.pdf --full_check -as --ref /state_groundtruth_estimate0
  evo_ape bag result.bag /state_groundtruth_estimate0 /dmvio/unscaled_pose -as
  evo_rpe bag result.bag /state_groundtruth_estimate0 /dmvio/unscaled_pose -as

  cd -
done
kill $ROSCORE_PID
