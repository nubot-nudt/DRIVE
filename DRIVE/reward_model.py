import os
import glob
import torch
import pickle
import itertools
import numpy as np
from torch import nn
from random import sample
import torch.nn.functional as F

from human_feedback_windows import evaluate_data

class RewardModelNetwork(nn.Module):
    def __init__(self, state_dim, action_dim,
                 features_dim = 64,
                 learning_rate = 5e-4,
                 weight_decay = 0.01,
                 dropout = 0.2,
                 ):
        super(RewardModelNetwork, self).__init__()
        self.learning_rate = learning_rate
        self.weight_decay = weight_decay

        self.fc_1 = nn.Linear(state_dim + action_dim, 500)
        self.fc_2 = nn.Linear(500, 300)
        self.fc_3 = nn.Linear(300, 1)
        self.sigmoid = nn.Sigmoid()
        self.relu = nn.ReLU()
        self.dropout = nn.Dropout(p=dropout)

        self.optimizer = torch.optim.Adam(self.parameters(), lr=self.learning_rate, weight_decay=self.weight_decay)

        self.rankloss = nn.MarginRankingLoss(margin=1.0)

    def forward(self, seq_obs, seq_actions):
        x = torch.cat([seq_obs, seq_actions], dim=-1)
        x = self.relu(self.fc_1(x))
        x = self.dropout(x)
        x = self.relu(self.fc_2(x))
        x = self.dropout(x)
        x = self.sigmoid(self.fc_3(x))
        return x

    def train_rp(self, batch_data):
        # 批量处理数据
        obs1, act1, obs2, act2, labels = [], [], [], [], []
        for data in batch_data:
            obs1.append(data[0][0])
            act1.append(data[0][1])
            obs2.append(data[1][0])
            act2.append(data[1][1])
            labels.append(data[2])
        # (obs1, act1), (obs2, act2), labels = zip(*batch_data)

        obs1 = torch.stack(obs1)
        act1 = torch.stack(act1)
        obs2 = torch.stack(obs2)
        act2 = torch.stack(act2)
        labels = torch.stack(labels)

        # y1_list, y2_list = [], []
        # for obs1_, obs2_, act1_, act2_ in zip(obs1, obs2, act1, act2):
        #     y1 = self(obs1_, act1_).squeeze()
        #     y2 = self(obs2_, act2_).squeeze()
        #     y1_list.append(y1)
        #     y2_list.append(y2)
        #
        # y1 = torch.stack(y1_list).squeeze()
        # y2 = torch.stack(y2_list).squeeze()

        y1 = self(obs1, act1).squeeze()
        y2 = self(obs2, act2).squeeze()

        loss = self.rankloss(y1, y2, labels)
        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()

        return loss.item()

class RewardModel:
    def __init__(self, state_dim, action_space, reward_model_n, Town='Town07', model_reward='', writer=None, data_type='image', dataset_save='reward_model_data/', save_reward_model_data=False):
        self.state_dim = state_dim
        self.action_dim = action_space.shape[0]
        self.reward_model_n = reward_model_n
        self.reward_models = [RewardModelNetwork(state_dim, self.action_dim) for _ in range(reward_model_n)]
        self.traj_with_label = [[], []]
        self.temp_experience = {
            "seq_obs": [],
            "seq_actions": [],
            "true_reward": [],
            "seq_distance": [],
            "seq_angle": [],
            "road_maneuver": []
        }
        if data_type == 'image':
            self.temp_experience["seq_viewer_image"] = []
        self.r_norm = RunningStat(shape = reward_model_n)
        self.step_it = 0
        self.train_it = 0
        self.label_num = 0
        self.model_reward = model_reward
        self.writer = writer
        self.Town = Town
        self.data_type = data_type
        self.dataset_save = dataset_save
        self.save_reward_model_data = save_reward_model_data
        self.save_it = 0
        self.rho = 0.05 # 0.05
        self.max_traj_samples = 2000

        self.maneuver_follow = 0
        self.maneuver_straight = 0
        self.maneuver_left = 0
        self.maneuver_right = 0

        self.temp_var = 0.996 ** 10 # 0.9607
        self.metric_ = 1.0

    def predict(self, seq_obs, seq_actions):
        # [model.eval() for model in self.reward_models]
        # with torch.no_grad():
        #     preds = torch.stack([p.forward(seq_obs.unsqueeze(0), seq_actions.unsqueeze(0)) for p in self.reward_models]).view(self.reward_model_n, -1)
        #     pred_reward = torch.mean(preds, dim = 0)
        #     pred_reward_std = torch.std(preds, dim = 0)
        # self.rho = self.rho * 0.999
        # return pred_reward[-1] + pred_reward_std[-1] * self.rho

        [model.eval() for model in self.reward_models]
        with torch.no_grad():
            preds = torch.stack([p.forward(seq_obs.unsqueeze(0), seq_actions.unsqueeze(0)) for p in self.reward_models]).view(self.reward_model_n, -1)
        self.rho = self.rho * 0.999
        return preds, self.rho

    def add_temp_experience(self, seq_obs, seq_actions, seq_reward, seq_distance_from_center, seq_angle_from_center,
                            seq_road_maneuver, seq_viewer_image=None):
        new_data = {
            "seq_obs": torch.FloatTensor(seq_obs),
            "seq_actions": torch.FloatTensor(seq_actions),
            "true_reward": torch.FloatTensor(seq_reward),
            "seq_distance": torch.FloatTensor(seq_distance_from_center),
            "seq_angle": torch.FloatTensor(seq_angle_from_center),
            "road_maneuver": torch.LongTensor([self._maneuver_to_id(seq_road_maneuver)])
        }

        for key in new_data:
            self.temp_experience[key].append(new_data[key])

        if self.data_type == 'image':
            self.temp_experience['seq_viewer_image'].append(seq_viewer_image.copy())

    def _maneuver_to_id(self, maneuver):
        # 转换方法封装
        maneuver_map = {
            "Left": 1, "Right": 2,
            "Straight": 3, "Follow Lane": 4
        }
        return maneuver_map.get(maneuver, 0)

    def reset_temp_experience(self):
        if self.save_reward_model_data:
            with open(self.dataset_save + 'temp_experience_{}.pkl'.format(self.save_it), 'wb') as f:
                pickle.dump(self.temp_experience, f)
            self.save_it += 1

        for key in self.temp_experience:
            # del self.temp_experience[key][:]
            self.temp_experience[key] = []

    def update(self, step_it = None, get_human_feedback_flag = True):
        # 获取人类反馈
        # k = int(len(self.temp_experience['seq_obs']) * 0.3) # 用30%的数据
        k = 500
        if get_human_feedback_flag:
            self.get_human_feedback(k, step_it)
        # 开始训练
        if len(self.traj_with_label) > 1:
            for i in range(10):
                self.train_it += 1
                loss = 0.0
                for j in range(2):
                    loss += self.train(j)

                # 日志记录
                if self.writer:
                    self.writer.add_scalar('Reward Model/Loss', loss, global_step=self.train_it)
                print('{} Reward Model Loss: {}'.format(self.train_it, loss))

        self.save(self.train_it)
        self.reset_temp_experience()

    def get_human_feedback(self, k, step_it = None):
        # if len(self.traj_with_label[0]) > self.max_traj_samples:
        #     remove_num = len(self.traj_with_label[0]) - self.max_traj_samples
        #     del self.traj_with_label[0][:remove_num]  # 删除最早的数据
        #     del self.traj_with_label[1][:remove_num]  # 删除最早的数据

        n = len(self.temp_experience['seq_obs'])
        k = min(k, int(n / 2))
        k = k - k % 2
        indices = torch.randperm(n)[:2 * k].view(k, 2)
        selected_pairs = [(i[0].item(), i[1].item()) for i in indices]

        print('Human Feedback Data Num: ', k)

        # 批量处理对比数据
        batch_data = []
        for i, j in selected_pairs:
            batch_data.append((
                (self.temp_experience['seq_obs'][i][0], self.temp_experience['seq_actions'][i][0]),
                (self.temp_experience['seq_obs'][j][0], self.temp_experience['seq_actions'][j][0]),
                self._generate_feedback(i, j)
            ))

        # 批量添加反馈
        for seg1, seg2, feedback in batch_data:
            if feedback is not None:
                if feedback[0].item() != 2:
                    self.traj_with_label[0].append((seg1, seg2, feedback[0]))
                if feedback[1].item() != 2:
                    self.traj_with_label[1].append((seg1, seg2, feedback[1]))

        self.metric_ = self.metric_ * self.temp_var
        if self.writer:
            self.label_num += k
            if step_it is None:
                self.step_it += 1
                step_it = self.step_it
            self.writer.add_scalar('Reward Model/Label Num', self.label_num, global_step=step_it)
            self.writer.add_scalar('Reward Model/Metric', self.metric_, global_step=step_it)

    def _generate_feedback(self, i, j):
        # 封装反馈生成逻辑
        distance_i = self.temp_experience['seq_distance'][i].item()
        distance_j = self.temp_experience['seq_distance'][j].item()
        speed_i = self.temp_experience['seq_obs'][i][0][67].item() * 3.6
        speed_j = self.temp_experience['seq_obs'][j][0][67].item() * 3.6

        fb = []
        # 距离中心线判断
        if min(distance_i, distance_j) < 1.0:# * self.metric_:
            fb.append(1 if distance_i < distance_j else -1)
        else:
            fb.append(2)
        # 速度保持判断
        if abs(speed_i - 12) < 3.0 or abs(speed_j - 12) < 3.0:# * self.metric_:
            fb.append(1 if abs(speed_i - 12) < abs(speed_j - 12) else -1)
        else:
            fb.append(2)

        return torch.tensor(fb)

        # if not fb:
        #     return None
        # return torch.tensor([sum(1 for x in fb if x == 1) / len(fb),
        #                      sum(1 for x in fb if x == 2) / len(fb)])

    def train(self, j):
        [model.train() for model in self.reward_models[j * 0 : (j + 1) * 3]]
        batch_size = min(1000, len(self.traj_with_label[j]))
        batch = sample(self.traj_with_label[j], batch_size)

        # 并行训练多个模型
        losses = []
        for model in self.reward_models:
            loss = model.train_rp(batch)
            losses.append(loss)

        return sum(losses) / len(losses)

    def save(self, it):
        state_dicts = {"model_{}".format(i): model.state_dict()
                       for i, model in enumerate(self.reward_models)}
        torch.save(state_dicts, os.path.join(self.model_reward, "reward_models_{}.pth".format(it)))

    def load(self, it):
        path = os.path.join(self.model_reward, "reward_models_{}.pth".format(it))
        if os.path.exists(path):
            state_dicts = torch.load(path)
            for i, model in enumerate(self.reward_models):
                model.load_state_dict(state_dicts["model_{}".format(i)])

    def load_data_all(self, data_name = "temp_experience.pkl"):
        paths = glob.glob(data_name + "*.pkl")
        for i in range(len(paths)):
            if i == len(paths) - 1 or i == len(paths) - 100 or i == len(paths) - 200:
                test = 0
            data_name = paths[i]
            with open(data_name, 'rb') as f:
                temp_experience = pickle.load(f)
            if len(temp_experience['seq_obs']) > 0:
                for (seq_obs,
                     seq_actions,
                     seq_road_maneuver,
                     seq_distance,
                     seq_viewer_image,
                     seq_angle,
                     true_reward) in zip(temp_experience['seq_obs'],
                                         temp_experience['seq_actions'],
                                         temp_experience['seq_road_maneuver'],
                                         temp_experience['seq_distance_from_center'],
                                         temp_experience['seq_viewer_image'],
                                         temp_experience['seq_angle_from_center'],
                                         temp_experience['true_reward']):
                    self.temp_experience['seq_obs'].append(seq_obs)
                    self.temp_experience['seq_actions'].append(seq_actions)
                    self.temp_experience['seq_road_maneuver'].append(seq_road_maneuver)
                    self.temp_experience['seq_distance_from_center'].append(seq_distance)
                    # self.temp_experience['seq_viewer_image'].append(seq_viewer_image)
                    self.temp_experience['seq_angle_from_center'].append(seq_angle)
                    self.temp_experience['true_reward'].append(true_reward)

    def load_data(self, data_name = "traj_with_label.pkl"):
        with open(data_name, 'rb') as f:
            traj_with_label_loaded = pickle.load(f)
        self.traj_with_label = traj_with_label_loaded

    def save_data(self, data_name = "traj_with_label.pkl"):
        with open(data_name, 'wb') as f:
            pickle.dump(self.traj_with_label, f)

class RunningStat(object):
    def __init__(self, shape=()):   # shape = 5
        self._n = 0
        self._M = torch.zeros(shape) # (5,)
        self._S = torch.zeros(shape) # (5,)

    def push(self, x: torch.Tensor): # TODO:没看懂
        # x = th.asarray(x)
        assert x.shape == self._M.shape
        self._n += 1
        if self._n == 1:
            self._M[...] = x
        else:
            oldM = self._M.clone()
            self._M[...] = oldM + (x - oldM)/self._n
            self._S[...] = self._S + (x - oldM)*(x - self._M)

    @property
    def n(self):
        return self._n

    @property
    def mean(self):
        return self._M

    @property
    def var(self):
        if self._n >= 2:
            return self._S/(self._n - 1)
        else:
            return torch.square(self._M)

    @property
    def std(self):
        return torch.sqrt(self.var)

    @property
    def shape(self):
        return self._M.shape
