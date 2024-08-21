#!/bin/bash

current_directory="$(cd "$(dirname "$0")" && pwd)"
project_root_dir=${current_directory}/../..

SEQUENCE_PATH=$1
SEQUENCE_NAME=$2
BAG_PATH=$3
GT_TOPIC_NAME=$4
CONFIG_FILE_PATH=$5

echo "SEQUENCE_PATH: $SEQUENCE_PATH"
echo "SEQUENCE_NAME: $SEQUENCE_NAME"
echo "BAG_PATH: $BAG_PATH"
echo "GT_TOPIC_NAME: $GT_TOPIC_NAME"
echo "CONFIG_FILE_PATH: $CONFIG_FILE_PATH"

cd $SEQUENCE_PATH
mkdir -p results/posegraph

# start the roscore
roscore &
ROSCORE_PID=$!

sleep 2

rosrun vins vins_node $CONFIG_FILE_PATH \
  > $SEQUENCE_NAME-vins-node.output 2>&1 &
VINS_NODE_PID=$!
rosrun loop_fusion loop_fusion_node $CONFIG_FILE_PATH \
  > $SEQUENCE_NAME-loop-fusion-node.output 2>&1 &
LOOP_FUSION_PID=$!

sleep 5

# for each result, we save the odometry result as a ros bag
echo "Start to record the odometry result"
# use output redirect function to avoid seeing the error message when try to kill the rosbag process
# https://github.com/ros/ros_comm/issues/2235
rosbag record -O odom_result /vins_estimator/odometry > rosbag.output 2>&1 &
ROSBAG_RECORD_PID=$!
sleep 1

echo "Start to play the dataset bag"
rosbag play ${BAG_PATH} &
ROSBAG_PLAY_PID=$!

CSV_FILE=system.csv
touch $SEQUENCE_PATH/$CSV_FILE

while ps -p $ROSBAG_PLAY_PID > /dev/null 2>&1; do
  TIMESTAMP=$(date +%s%3N)
  VINS_NODE_CPU_MEM_USAGE=$(ps -p $VINS_NODE_PID -o %cpu,%mem --no-headers)
  LOOP_FUSION_CPU_MEM_USAGE=$(ps -p $LOOP_FUSION_PID -o %cpu,%mem --no-headers)

  CPU1=$(echo $VINS_NODE_CPU_MEM_USAGE | awk '{print $1}')
  MEM1=$(echo $VINS_NODE_CPU_MEM_USAGE | awk '{print $2}')

  CPU2=$(echo $LOOP_FUSION_CPU_MEM_USAGE | awk '{print $1}')
  MEM2=$(echo $LOOP_FUSION_CPU_MEM_USAGE | awk '{print $2}')

  TOTAL_CPU=$(echo "$CPU1 + $CPU2" | bc)
  TOTAL_MEM=$(echo "$MEM1 + $MEM2" | bc)

  if [[ -n "$TOTAL_CPU" && -n "$TOTAL_MEM" ]]; then
    echo "$TIMESTAMP $TOTAL_CPU $TOTAL_MEM" >> "$CSV_FILE"
  fi

  sleep 0.1
done

sleep 5
echo "Killing rosbag process"
while kill -SIGINT $ROSBAG_RECORD_PID 2>/dev/null; do
  echo "Waiting for process $ROSBAG_RECORD_PID to be killed..."
  sleep 1
done
echo "Killing vins-fusion ros process"
kill $VINS_NODE_PID
kill $LOOP_FUSION_PID
kill $ROSCORE_PID

python3 ${current_directory}/python/cal_rosbag_frequency.py odom_result.bag /vins_estimator/odometry > odom_result.hz

# merge the bags of result and grount truth into one bag
python3 ${current_directory}/python/bagmerge.py -o result.bag -t $GT_TOPIC_NAME /vins_estimator/odometry -i odom_result.bag $BAG_PATH

evo_traj bag result.bag /vins_estimator/odometry --save_plot plot.pdf --full_check -as --ref $GT_TOPIC_NAME
evo_ape bag result.bag $GT_TOPIC_NAME /vins_estimator/odometry -as
evo_rpe bag result.bag $GT_TOPIC_NAME /vins_estimator/odometry -as

cd -