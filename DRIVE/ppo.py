import os
import torch
import numpy as np
import scipy.signal
import torch.nn as nn
import torch.optim as optim
from torch.distributions import Normal

class ActorCritic(nn.Module):
    def __init__(self, input_dim, action_space, layer_norm = True, action_std_init=0.1):
        super(ActorCritic, self).__init__()
        self.relu = nn.ReLU()
        self.action_dim = action_space.shape[0]
        self.steer_clip = torch.Tensor([action_space.high[0]])
        self.throttle_clip = torch.Tensor([action_space.high[1]]) / 2.0
        self.fc_1 = nn.Linear(input_dim, 500)
        self.fc_2 = nn.Linear(500, 300)
        self.fc_mu = nn.Linear(300, self.action_dim)
        self.action_logstd = torch.nn.Parameter(
            torch.full((self.action_dim,), np.log(action_std_init), dtype=torch.float32),
            requires_grad=True  # 默认为 True，表示该参数是可训练的
        )
        self.tanh = nn.Tanh()

        # Critic
        self.critic_fc1 = nn.Linear(input_dim, 500)
        self.critic_fc2 = nn.Linear(500, 300)
        self.critic_fc3 = nn.Linear(300, 1)

        if layer_norm:
            self.layer_norm(self.fc_1, std=1.0)
            self.layer_norm(self.fc_2, std=1.0)
            self.layer_norm(self.fc_mu, std=0.01)

            self.layer_norm(self.critic_fc1, std=1.0)
            self.layer_norm(self.critic_fc2, std=1.0)
            self.layer_norm(self.critic_fc3, std=1.0)

    @staticmethod
    def layer_norm(layer, std=1.0, bias_const=0.0):
        torch.nn.init.orthogonal_(layer.weight, std)
        torch.nn.init.constant_(layer.bias, bias_const)

    def reset_action_std(self, new_std):
        self.action_logstd = torch.nn.Parameter(
            torch.full((self.action_dim,), np.log(new_std), dtype=torch.float32),
            requires_grad=True  # 默认为 True，表示该参数是可训练的
        )

    def forward(self, states):
        return self._forward_actor(states).detach().numpy()

    def select_action(self, states):
        action_mean = self._forward_actor(states)
        value = self._forward_critic(states)
        action_std = torch.exp(self.action_logstd)
        normal = Normal(action_mean, action_std)
        # x_t = normal.rsample()
        # action = self.tanh(x_t)
        action = normal.rsample()
        logprob = normal.log_prob(action)
        return action, value[0], logprob.sum(), action_mean, action_std

    def get_logprob(self, states, actions):
        action_mean = self._forward_actor(states)
        action_std = torch.exp(self.action_logstd)
        normal = Normal(action_mean, action_std)
        logprob = normal.log_prob(actions)
        return logprob.sum(1)

    def _forward_actor(self, x):
        x = self.relu(self.fc_1(x))
        x = self.relu(self.fc_2(x))
        mu = self.tanh(self.fc_mu(x))
        return mu

    def _forward_critic(self, x):
        x = torch.relu(self.critic_fc1(x))
        x = torch.relu(self.critic_fc2(x))
        val = self.critic_fc3(x)
        return val

class PPO(nn.Module):
    def __init__(self,
                 input_shape = None,
                 action_space = None,
                 learning_rate = 1e-4,
                 learning_rate_min = 1e-6,
                 clip_norm = False,
                 loss_value_norm = False,
                 grad_clip = False,
                 advantage_norm = True,
                 layer_norm = True,
                 reward_centering = True,
                 desired_kl = None,
                 discount_factor = 0.98,
                 gae_lambda = 0.95,
                 ppo_epsilon = 0.2,
                 value_coef = 1.0,
                 entropy_coef = 0.05,
                 ppo_model = 'ppo/',
                 action_std_init = 1.0,
                 ):
        super(PPO, self).__init__()
        self.lr = learning_rate
        self.clip_norm = clip_norm
        self.loss_value_norm = loss_value_norm
        self.grad_clip = grad_clip
        self.advantage_norm = advantage_norm
        self.reward_centering = reward_centering
        self.discount_factor = discount_factor
        self.gae_lambda = gae_lambda
        self.ppo_epsilon = ppo_epsilon
        self.value_coef = value_coef
        self.entropy_coef = entropy_coef
        self.desired_kl = desired_kl
        self.model_dir = ppo_model  # 模型路径

        self.policy = ActorCritic(input_shape, action_space, layer_norm, action_std_init)
        self.optimizer = optim.Adam(self.policy.parameters(), lr=learning_rate, eps=learning_rate_min)

    def predict(self, state):
        state = torch.tensor(state, dtype=torch.float32)
        action, value, logprob, action_mean, action_std = self.policy.select_action(state)
        return action.detach().numpy(), value.detach().numpy(), logprob.detach().numpy(), action_mean.detach().numpy(), action_std.detach().numpy()

    def action_inference(self, state):
        state = torch.tensor(state, dtype=torch.float32)
        return self.policy.forward(state)

    def update(self, states, actions, rewards, next_states, dones, logprobs, values, last_value,
               batch_size, minibatch_size, network_update_epochs, writer, step_it, total_steps):
        # 计算adv法一
        rewards = np.array(rewards)
        values = np.array(values + [last_value])
        terminals = np.array(dones)
        deltas = rewards + terminals * self.discount_factor * values[1:] - values[:-1]
        advantages = scipy.signal.lfilter([1], [1, -self.discount_factor * self.gae_lambda], deltas[::-1], axis=0)[::-1]
        returns = advantages + values[:-1]

        batch_returns = torch.Tensor(returns)
        batch_advantages = torch.Tensor(advantages.copy())
        batch_states = torch.Tensor(np.array(states))
        batch_actions = torch.Tensor(np.array(actions))
        batch_logprobs = torch.Tensor(np.array(logprobs))
        batch_values = torch.Tensor(np.array(values)[:-1])

        # 优势函数标准化
        if self.advantage_norm:
            batch_advantages = (batch_advantages - batch_advantages.mean()) / (batch_advantages.std() + 1e-8)

        # 开始更新
        loss_surr_ = []
        loss_value_ = []
        loss_entropy_ = []
        loss_ = []
        for _ in range(int(network_update_epochs * batch_size // minibatch_size)):
            # 采样
            minibatch_idx = np.random.choice(len(batch_states), minibatch_size, replace=False)
            minibatch_states = batch_states[minibatch_idx]
            minibatch_actions = batch_actions[minibatch_idx]
            minibatch_logprobs = batch_logprobs[minibatch_idx]
            minibatch_advantages = batch_advantages[minibatch_idx]
            minibatch_returns = batch_returns[minibatch_idx]
            minibatch_old_values = batch_values[minibatch_idx]

            minibatch_new_logprob = self.policy.get_logprob(minibatch_states, minibatch_actions)
            minibatch_new_values = self.policy._forward_critic(minibatch_states)

            # 策略网络损失
            ratio = torch.exp(minibatch_new_logprob - minibatch_logprobs)
            surr1 = ratio * minibatch_advantages
            surr2 = ratio.clamp(1 - self.ppo_epsilon, 1 + self.ppo_epsilon) * minibatch_advantages
            loss_surr = -torch.mean(torch.min(surr1, surr2))

            # 价值网络损失
            minibatch_return_6std = 6 * minibatch_returns.std()
            minibatch_new_values = minibatch_new_values.squeeze()
            if self.loss_value_norm:
                value_clipped = minibatch_old_values + (minibatch_new_values - minibatch_old_values).clamp(
                    -self.ppo_epsilon, self.ppo_epsilon)
                loss_value = (minibatch_new_values - minibatch_returns).pow(2)
                loss_value_clipped = (value_clipped - minibatch_returns).pow(2)
                loss_value = torch.max(loss_value, loss_value_clipped).mean()
            else:
                loss_value = torch.mean((minibatch_new_values - minibatch_returns).pow(2)) / (
                            minibatch_return_6std + 1e-8)

            # 熵损失
            loss_entropy = torch.mean(torch.exp(minibatch_new_logprob) * minibatch_new_logprob)

            # 总损失
            loss = loss_surr + self.value_coef * loss_value + self.entropy_coef * loss_entropy

            # 反向传播
            self.optimizer.zero_grad()
            loss.backward()
            if self.grad_clip:
                torch.nn.utils.clip_grad_norm_(self.policy.parameters(), 1.0)
            self.optimizer.step()

            loss_surr_.append(loss_surr.detach().numpy())
            loss_value_.append(loss_value.detach().numpy())
            loss_entropy_.append(loss_entropy.detach().numpy())
            loss_.append(loss.detach().numpy())

        if self.clip_norm:
            ratio = 1 - step_it / total_steps
            self.ppo_epsilon = self.ppo_epsilon * ratio
            writer.add_scalar('Network/Clip', self.ppo_epsilon, step_it)
        if not self.desired_kl:
            ratio = 1 - 0.2 * step_it / total_steps
            lr_new = self.lr * ratio
            writer.add_scalar('Network/Learning Rate', lr_new, step_it)
            for g in self.optimizer.param_groups:
                g['lr'] = lr_new

        writer.add_scalar('Network/Policy Loss', np.mean(loss_surr_), step_it)
        writer.add_scalar('Network/Entropy Loss', np.mean(loss_entropy_), step_it)
        writer.add_scalar('Network/Value Loss', np.mean(loss_value_), step_it)
        writer.add_scalar('Network/Total Loss', np.mean(loss_), step_it)

    def save(self, it):
        torch.save(self.policy.state_dict(), os.path.join(self.model_dir, "model_{}.pth".format(it)))
        print("Model checkpoint save.")

    def load(self, it):
        self.policy.load_state_dict(torch.load(os.path.join(self.model_dir, "model_{}.pth".format(it))))
        print("Model checkpoint load.")