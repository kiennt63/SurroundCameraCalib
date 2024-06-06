#!/bin/bash

in_dir=$1
out_dir=$2

export LD_LIBRARY_PATH=/opt/opencv/opencv-3.4.5/lib:/opt/mpc/3rdparty/protobuf/lib/x64_linux

for d in $(find ${in_dir}/* -type d); do
    echo "Running calibration for $d"
    timestamp=$(basename $d)
    mkdir -p ${out_dir}/${timestamp}
    ./bin/run_AVM_Calibration_F custom /home/kiennt63/data/calibration/vf8/output ${d} ${out_dir}/${timestamp}
done
