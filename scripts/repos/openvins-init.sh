#!/bin/bash
current_directory="$(cd "$(dirname "$0")" && pwd)"
project_root_dir=${current_directory}/../..

mkdir -p ${project_root_dir}/workspace/src && cd ${project_root_dir}/workspace/src

git clone https://github.com/rpng/open_vins/