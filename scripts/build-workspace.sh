#!/bin/bash
current_directory="$(cd "$(dirname "$0")" && pwd)"
project_root_dir=${current_directory}/..

bash -c "cd ${project_root_dir}/workspace &&
        export DMVIO_BUILD=${project_root_dir}/workspace/src/dm-vio/build &&
        source /opt/ros/noetic/setup.bash && 
        catkin build &&
        echo \"source ${project_root_dir}/workspace/devel/setup.bash\" >> ~/.bashrc"

