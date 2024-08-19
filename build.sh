
#!/bin/bash
current_directory="$(cd "$(dirname "$0")" && pwd)"
project_root_dir=${current_directory}

WORKSPACE_DIR=/root/VIO-auto-tests

cd ${project_root_dir} && docker build -t ghcr.io/dibuli/vio-autonomous-test:latest .