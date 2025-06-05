#!/bin/bash

gnome-terminal --tab -- bash -c "roscore"
sleep 1s
gnome-terminal --tab -- bash -c "cd ~/Coppeliasim_Carla/Coppeliasim/src/CoppeliaSim_Edu_V4_1_0_Ubuntu20_04;./coppeliaSim.sh"
sleep 30s
gnome-terminal --tab -- bash -c "cd ~/Coppeliasim_Carla/Robcup_2022/TL_Code;source devel/setup.bash;rosrun r7_auto_sim r7_auto_sim"
sleep 1s
gnome-terminal --tab -- bash -c "cd ~/Coppeliasim_Carla/carla_0_9_5;./CarlaUE4.sh -quality_level=Low -benchmark -fps=15 -windowed -ResX=128 -ResY=64"
sleep 5s
gnome-terminal --tab -- bash -c "source ~/anaconda3/etc/profile.d/conda.sh;conda activate carla_py35;cd ~/Coppeliasim_Carla/DRIVE/;python train.py;read"
