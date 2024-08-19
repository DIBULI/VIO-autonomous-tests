#!/bin/bash
current_directory="$(cd "$(dirname "$0")" && pwd)"
project_root_dir=${current_directory}/../..

RESULT_FOLDER_PATH=$1
SEQUENCE_NAME=$2
BAG_PATH=$3
GT_TOPIC_NAME=$4

cd $RESULT_FOLDER_PATH

# start the roscore
roscore &
ROSCORE_PID=$!

sleep 2

# camera intrinsic
echo -e "458.654 457.296 367.215 248.375 -0.28340811 0.07395907 0.00019359 1.76187114e-05\n752 480\ncrop\n640 480\n" > camera.txt

rosrun dmvio_ros node calib=$RESULT_FOLDER_PATH/camera.txt \
  settingsFile=$DMVIO_BUILD/../configs/euroc.yaml \
  mode=1 nogui=1 preset=1 useimu=1 quiet=0 init_requestFullResetNormalizedErrorThreshold=0.8 init_pgba_skipFirstKFs=1 \
  > dmvio.output 2>&1 &
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
rosbag play ${BAG_PATH} &
ROSBAG_PLAY_PID=$!

CSV_FILE=system.csv
touch $CSV_FILE

while ps -p $ROSBAG_PLAY_PID > /dev/null 2>&1; do
  TIMESTAMP=$(date +%s%3N)
  CPU_MEM_USAGE=$(ps -p $DMVIO_ROS_PID -o %cpu,%mem --no-headers)

  if [ -n "$CPU_MEM_USAGE" ]; then
      echo "$TIMESTAMP $CPU_MEM_USAGE" >> "$CSV_FILE"
  fi

  sleep 0.1
done

sleep 3
echo "Killing rosbag process"
while kill -SIGINT $ROSBAG_RECORD_PID 2>/dev/null; do
  echo "Waiting for process $ROSBAG_RECORD_PID to be killed..."
  sleep 2
done
echo "Killing dmvio ros process"
kill $DMVIO_ROS_PID
kill $ROSCORE_PID

python3 ${current_directory}/python/cal_rosbag_frequency.py odom_result.bag /dmvio/unscaled_pose > odom_result.hz

# merge the bags of result and grount truth into one bag
python3 ${current_directory}/python/bagmerge.py -o result.bag -t $GT_TOPIC_NAME /dmvio/unscaled_pose -i odom_result.bag $BAG_PATH

evo_traj bag result.bag /dmvio/unscaled_pose --save_plot plot.pdf --full_check -as --ref $GT_TOPIC_NAME
evo_ape bag result.bag $GT_TOPIC_NAME /dmvio/unscaled_pose -as
evo_rpe bag result.bag $GT_TOPIC_NAME /dmvio/unscaled_pose -as

cd -