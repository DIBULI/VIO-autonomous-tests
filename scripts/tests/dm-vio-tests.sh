#!/bin/bash
current_directory="$(cd "$(dirname "$0")" && pwd)"
project_root_dir=${current_directory}/../..


DATASET_PATH=$DATASET_PATH
TEST_RESULT_PATH=${project_root_dir}/tests_result

mkdir -p ${TEST_RESULT_PATH} && cd ${TEST_RESULT_PATH}

mkdir -p dm-vio-tests && cd dm-vio-tests

# camera intrinsic


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
    done < <(find "${EUROC_DATASET_PATH}/$SCENRIOS" -type f -print0)
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

  rosrun dmvio_ros node calib=${TEST_RESULT_PATH}/dm-vio-tests/euroc/${SEQUENCE_NAME}/camera.txt \
    settingsFile=$DMVIO_BUILD/../configs/euroc.yaml \
    mode=1 nogui=1 preset=1 useimu=1 quiet=1 init_requestFullResetNormalizedErrorThreshold=0.8 init_pgba_skipFirstKFs=1 \
    rosbag=${BAG} loadRosbagThread=1 > ${SEQUENCE_NAME}.output &
  
  DMVIO_ROS_PID=$!

  CSV_FILE=euroc/${SEQUENCE_NAME}/system.csv
  mkdir -p euroc/${SEQUENCE_NAME}
  touch ${CSV_FILE}
  while ps -p $DMVIO_ROS_PID > /dev/null 2>&1; do
    TIMESTAMP=$(date +%s%3N)
    CPU_MEM_USAGE=$(ps -p $DMVIO_ROS_PID -o %cpu,%mem --no-headers)

    if [ -n "$CPU_MEM_USAGE" ]; then
        echo "$TIMESTAMP $CPU_MEM_USAGE" >> "$CSV_FILE"
    fi

    sleep 0.1
  done
done
kill $ROSCORE_PID

