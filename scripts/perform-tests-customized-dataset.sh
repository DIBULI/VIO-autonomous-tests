#!/bin/bash
current_directory="$(cd "$(dirname "$0")" && pwd)"
project_root_dir=${current_directory}/..

DATASET_PATH="${DATASET_PATH:-/data}"
TEST_RESULT_PATH="${TEST_RESULT_PATH:-/root/VIO-auto-tests/test-results}"

BAG_PATH=$1
CONFIG_PATH=$2
GT_TOPIC_NAME=$3

WORKSPACE=${project_root_dir}/workspace

source ${WORKSPACE}/devel/setup.bash

mkdir -p $TEST_RESULT_PATH && cd $TEST_RESULT_PATH

# vins-fusion tests
rm -rf vins-fusion-tests
mkdir -p vins-fusion-tests
cd vins-fusion-tests

echo "-------------------------- processing dataset: $BAG_PATH ------------------------------"
SEQUENCE_NAME_WITH_EXT=$(basename "$BAG_PATH")
SEQUENCE_NAME="${SEQUENCE_NAME_WITH_EXT%.*}"

mkdir -p $TEST_RESULT_PATH/vins-fusion-tests/$SEQUENCE_NAME

bash ${current_directory}/tests/vins-fusion-tests.sh $TEST_RESULT_PATH/vins-fusion-tests/$SEQUENCE_NAME \
    $SEQUENCE_NAME $BAG_PATH $GT_TOPIC_NAME $CONFIG_PATH
cd -