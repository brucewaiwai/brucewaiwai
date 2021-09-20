#!/bin/bash

base_path="/home/tmrobot/Desktop/map"
if [ ! -d $base_path ]; then
    mkdir $base_path
fi
rosservice call /write_state "filename: '${base_path}/cartographer.pbstream'"
sleep 1
cd $base_path
rosrun cartographer_ros cartographer_pbstream_to_ros_map -pbstream_filename ${base_path}/cartographer.pbstream -map_filestem loc nav -resolution 0.05
# rosrun map:=/mapping_map
cp loc.yaml nav.yaml
sed -i 's/image: loc.pgm/image: nav.pgm/g' nav.yaml
cp loc.pgm nav.pgm


# sed -i '1 d' 1.yaml
# sed -i '1 i\nav_image : 1_nav.pgm' 1.yaml
# sed -i '1 i\loc_image : 1_loc.pgm' 1.yaml
# cp 1.pgm 1_loc.pgm
# mv 1.pgm 1_nav.pgm

eog loc.pgm
