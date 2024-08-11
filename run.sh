
#!/bin/bash
current_directory="$(cd "$(dirname "$0")" && pwd)"
project_root_dir=${current_directory}

DATA_PATH=~/data
DATASET_PATH=${DATA_PATH}/dataset/vios
TEST_RESULT_FOLDER=~/VIOs-auto-test-results
DOCKER_RESULT_DIR=/root/VIO-auto-tests

sudo rm -r $TEST_RESULT_FOLDER

docker run -v ${DATASET_PATH}:/data -v ${TEST_RESULT_FOLDER}:${DOCKER_RESULT_DIR}/tests_result ghcr.io/dibuli/vio-autonomous-test:latest
