#!/bin/bash
python run_eval.py \
    --hdf5_path /home/ubuntu/data_ft/2/robot_data_20250811_004219.hdf5 \
    --trunk_size 3200 \
    --max_steps 600\
    --use_relative_actions false\
    --playback_speed 2.0\

