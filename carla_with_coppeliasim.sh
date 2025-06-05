#!/bin/bash

gnome-terminal --tab -- bash -c "roscore"
sleep 1s
gnome-terminal --tab -- bash -c "cd Coppeliasim;cd src;cd CoppeliaSim_Edu_V4_1_0_Ubuntu20_04;./coppeliaSim.sh"
sleep 30s
gnome-terminal --tab -- bash -c "cd Robcup_2022;cd TL_Code;source devel/setup.bash;rosrun r7_auto_sim r7_auto_sim"
sleep 1s
gnome-terminal --tab -- bash -c "cd carla_0_9_5;./CarlaUE4.sh -quality_level=Low -benchmark -fps=15 -windowed -ResX=128 -ResY=64"
sleep 5s
# gnome-terminal --tab -- bash -c "source /home/luo/anaconda3/etc/profile.d/conda.sh;conda activate carla_py35;cd Coppeliasim_carla/Carla-ppo/;python train.py;read"
gnome-terminal --tab -- bash -c "source /home/luo/anaconda3/etc/profile.d/conda.sh;conda activate carla_py35;cd DRIVE/;python train.py;read"
