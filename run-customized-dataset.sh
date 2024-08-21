#!/bin/bash
current_directory="$(cd "$(dirname "$0")" && pwd)"
project_root_dir=${current_directory}

TEST_RESULT_FOLDER=~/VIOs-auto-test-results-customized-dataset
WORKSPACE_DIR=/root/VIO-auto-tests

BAG_PATH=$1
BAG_PARENT_FOLDER=$(dirname $BAG_PATH)
BAG_NAME=$(basename "$BAG_PATH")
CONFIG_NAME=$2
GT_TOPIC_NAME=$3

sudo rm -r $TEST_RESULT_FOLDER

docker run -v $BAG_PARENT_FOLDER:/data -v ${TEST_RESULT_FOLDER}:${WORKSPACE_DIR}/test-results \
  -v ${project_root_dir}/configs:$WORKSPACE_DIR/configs \
  --entrypoint=/bin/bash ghcr.io/dibuli/vio-autonomous-test:latest \
  -i /root/VIO-auto-tests/scripts/perform-tests-customized-dataset.sh \
  /data/$BAG_NAME $WORKSPACE_DIR/configs/$CONFIG_NAME $GT_TOPIC_NAME
