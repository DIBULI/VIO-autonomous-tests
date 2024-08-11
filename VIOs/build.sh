
#!/bin/bash
current_directory="$(cd "$(dirname "$0")" && pwd)"
project_root_dir=${current_directory}

DATASET_PATH=~/data/dataset/vios
TEST_RESULT_FOLDER=~/VIOs-auto-test-results

WORKSPACE_DIR=/root/VIO-auto-tests

cd ${project_root_dir} && docker build -t ghcr.io/dibuli/vio-autonomous-test:latest .