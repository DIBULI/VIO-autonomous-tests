#!/bin/bash
current_directory="$(cd "$(dirname "$0")" && pwd)"
project_root_dir=${current_directory}/..

bash -c "bash ${current_directory}/repos/dm-vio-init.sh"