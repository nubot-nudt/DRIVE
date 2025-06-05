source /home/luo/anaconda3/etc/profile.d/conda.sh
conda activate carla_py35
export PYTHONPATH=$(echo $PYTHONPATH | tr ':' '\n' | grep '/opt/ros/noetic/' | paste -sd ':' -)
tensorboard --logdir ppo/logs/
