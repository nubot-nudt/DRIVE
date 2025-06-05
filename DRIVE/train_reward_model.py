import os
import numpy as np

from reward_model import RewardModel
from tensorboardX import SummaryWriter
from parameters import *

input_shape = Z_DIM + len(measurements_to_include) + 2 * n_points - 1
reward_model_n = 5 # REWARD_MODEL_N
Town = TOWN
ppo_model = PPO_MODEL
model_logs = ppo_model + 'logs/'
model_reward = ppo_model + 'reward_models/'
model_reward_model_data = ppo_model + DATASET_SAVE
data_type = DATA_TYPE
save_reward_model_data = False
action_space = np.array([0, 0])

if not os.path.exists(model_logs):
    os.makedirs(model_logs)
if not os.path.exists(model_reward):
    os.makedirs(model_reward)

writer = SummaryWriter(model_logs)

reward_model = RewardModel(input_shape, action_space, reward_model_n, Town,
                                   model_reward=model_reward,
                                   writer=writer,
                                   data_type=data_type,
                                   dataset_save=model_reward_model_data,
                                   save_reward_model_data=save_reward_model_data)
# 直接加载已标注数据训练RLHF
reward_model.load_data_all(ppo_model + DATASET_SAVE)
reward_model.update_direct2() # get_human_feedback_flag=False