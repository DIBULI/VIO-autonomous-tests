#!/bin/bash
current_directory="$(cd "$(dirname "$0")" && pwd)"
project_root_dir=${current_directory}/../..

# https://cvg.cit.tum.de/data/datasets/visual-inertial-dataset
wget -R "index.*" -m -np -nH --no-check-certificate -e robots=off \
    https://cdn2.vision.in.tum.de/tumvi/exported/euroc/512_16/

# optionally verify md5 sums:
cd tumvi/exported/euroc/512_16
md5sum -c *.md5