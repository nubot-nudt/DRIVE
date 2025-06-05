import os
import ast
import time
import torch
import argparse
import numpy as np
from tensorboardX import SummaryWriter

from ppo import PPO
from parameters import *
from run_eval import run_eval
from vae.models import VAE, AE, VAE_torch
from vae.models_tf import ConvVAE
from reward_model import RewardModel
from reward_functions import reward_func
from vae_common import create_encode_state_fn, create_encode_state_fn_tf, create_encode_state_fn_torch, Normalization, RewardScaling

USE_ROUTE_ENVIRONMENT = False
if USE_ROUTE_ENVIRONMENT:
    from CarlaEnv.carla_route_env import CarlaRouteEnv as CarlaEnv
else:
    from CarlaEnv.carla_lap_env import CarlaLapEnv as CarlaEnv

def load_vae(model_type, model_dir, z_dim, model_it):
    if model_type == 'VAE':
        vae = VAE(latent_size=z_dim) # VAE网络初始化
    elif model_type == 'AE':
        vae = AE(latent_size=z_dim)

    temp_var = model_dir + str(model_it) + '.pth'
    print('load vae model: ', temp_var)
    vae.load(temp_var) # 加载VAE模型
    return vae

def load_vae_tf(model_dir, z_dim=None):
    vae = ConvVAE(source_shape=np.array([80, 160, 3]),
                   target_shape=np.array([80, 160, 1]),
                   z_dim=z_dim,
                   model_dir=model_dir,
                   training=False)
    vae.init_session(init_logging=False)
    vae.load_latest_checkpoint()
    return vae

def load_vae_torch(vae_dir, latent_size):
    model_dir = os.path.join(vae_dir, 'best.pth')
    model = VAE_torch(latent_size)
    model.load_state_dict(torch.load(model_dir))
    return model

def train(params, it):
    if True:
        reward_model_n = params["reward_model_n"]
        seq_len = params["seq_len"]
        data_type = params["data_type"]
        dataset_save = params["dataset_save"]
        save_reward_model_data = params["save_reward_model_data"]
        use_rlhf = params["use_rlhf"]

        model_type = params["model_type"]
        vae_model = params["vae_model"]
        checkpoint = params["checkpoint"]
        
        learning_rate = params["learning_rate"]
        learning_rate_min = params["learning_rate_min"]
        desired_kl = params["desired_kl"]
        clip_norm = params["clip_norm"]
        loss_value_norm = params["loss_value_norm"]
        use_reward_scaling = params["reward_scaling"]
        use_reward_centering = params["reward_centering"]
        use_reward_normalization = params["reward_normalization"]
        grad_clip = params["grad_clip"]
        advantage_norm = params["advantage_norm"]
        layer_norm = params["layer_norm"]
        discount_factor = params["discount_factor"]
        gae_lambda = params["gae_lambda"]
        ppo_epsilon = params["ppo_epsilon"]
        value_coef = params["value_scale"]
        entropy_coef = params["entropy_scale"]
        ppo_model = params["ppo_model"]
        action_std_init = params["action_std_init"]
        use_state_norm = params["state_norm"]
        network_update_epochs = params["network_update_epochs"]
        batch_size = params["batch_size"]
        minibatch_size = params["minibatch_size"]
        action_smoothing = params["action_smoothing"]

        activate_render = params["activate_render"]
        start_carla = params["start_carla"]
        synchronous = params["synchronous"]
        fps = params["fps"]
        if_ros = params["if_ros"]
        Town = params["Town"]

        break_steps = params["break_steps"]
        eval_interval = params["eval_interval"]
        total_steps = params["total_steps"]
        episode_length = params["episode_length"]
        record_to_file = params["record_to_file"]

        model_logs = ppo_model + 'logs/' + str(it) + '/'
        model_ppo = ppo_model + 'models/' + str(it) + '/'
        model_reward = ppo_model + 'reward_models/' + str(it) + '/'
        model_reward_model_data = ppo_model + dataset_save
        if not os.path.exists(model_logs):
            os.makedirs(model_logs)
        if not os.path.exists(model_ppo):
            os.makedirs(model_ppo)
        if not os.path.exists(record_to_file):
            os.makedirs(record_to_file)
        if not os.path.exists(model_reward_model_data):
            os.makedirs(model_reward_model_data)
        if use_rlhf:
            if not os.path.exists(model_reward):
                os.makedirs(model_reward)

        writer = SummaryWriter(model_logs)

        train_rlhf = False

    # VAE-torch
    # vae = load_vae(model_type, vae_model, Z_DIM, checkpoint)
    # encode_state_func = create_encode_state_fn(model_type, vae, measurements_to_include)

    if Town == 'Town02':
        # VAE-torch-V2
        vae = load_vae_torch(vae_model, Z_DIM)
        encode_state_func = create_encode_state_fn_torch(vae, measurements_to_include)
    else:
        # VAE-tensorflow
        vae = load_vae_tf(vae_model, Z_DIM)
        encode_state_func = create_encode_state_fn_tf(vae, measurements_to_include)

    # 状态维度
    input_shape = Z_DIM + len(measurements_to_include) + 2 * n_points - 1

    # 环境初始化
    env = CarlaEnv(obs_res=(160, 80),
                   fps=fps,
                   activate_render=activate_render,
                   start_carla=start_carla,
                   synchronous=synchronous,
                   if_ros=if_ros,
                   Town=Town,

                   vae=vae,
                   
                   action_smoothing=action_smoothing,
                   encode_state_fn=encode_state_func,
                   reward_fn=reward_func,

                   writer=writer,
                   
                   use_rlhf=use_rlhf,
                   state_dim=input_shape,
                   seq_len=seq_len,
                   data_type=data_type,
                   save_reward_model_data=save_reward_model_data,
                   it=it,
                   )

    # RLHF初始化
    if use_rlhf:
        reward_model = RewardModel(input_shape, 
                                   env.action_space, 
                                   reward_model_n, 
                                   Town,
                                   model_reward=model_reward,
                                   writer=writer,
                                   data_type=data_type,
                                   dataset_save=model_reward_model_data,
                                   save_reward_model_data=save_reward_model_data)
        train_rlhf = True
        reward_model.load(0)
        # 环境再初始化
        env.reward_model_init(reward_model)

    # PPO
    if use_state_norm:
        state_norm = Normalization(shape=input_shape)
    if use_reward_normalization:
        reward_norm = Normalization(shape=1)
    if use_reward_scaling:
        reward_scaling = RewardScaling(shape=1, gamma=discount_factor)

    model = PPO(input_shape, env.action_space,
                learning_rate=learning_rate,
                learning_rate_min=learning_rate_min,
                clip_norm=clip_norm,
                loss_value_norm=loss_value_norm,
                grad_clip=grad_clip,
                advantage_norm=advantage_norm,
                layer_norm=layer_norm,
                reward_centering=use_reward_centering,
                desired_kl=desired_kl,
                discount_factor=discount_factor,
                gae_lambda=gae_lambda,
                ppo_epsilon=ppo_epsilon,
                value_coef=value_coef,
                entropy_coef=entropy_coef,
                ppo_model=model_ppo,
                action_std_init=action_std_init,
                )  # PPO初始化

    step_it = 0
    test_it = 0
    episode_idx = 0
    max_train_reward = 0
    max_eval_reward = 0
    while step_it < total_steps:
        states, actions, rewards, next_states, dones, logprobs, values = [], [], [], [], [], [], []

        # if step_it > 120000:
        #     use_rlhf = False

        while len(states) < batch_size:
            episode_idx += 1
            state, total_reward, done, total_predict_reward = env.reset(), 0, False, 0
            if use_state_norm:
                state = state_norm(state)
            if use_reward_scaling:
                reward_scaling.reset()

            # 仿真异常处理
            mini_states, mini_actions, mini_rewards, mini_next_states, mini_dones, mini_logprobs, mini_values = [], [], [], [], [], [], []
            simulation_error = False
            vehicle_init_pos = [env.vehicle.get_transform().location.x, env.vehicle.get_transform().location.y]
            for it in range(episode_length):
                step_it += 1
                action, value, logprob, action_mean, action_std = model.predict(state)
                next_state, reward, done, predict_reward = env.step(action, it)
                total_reward += reward
                total_predict_reward += predict_reward

                if use_state_norm:
                    next_state = state_norm(next_state)
                if use_reward_normalization:
                    reward = reward_norm(reward)
                elif use_reward_scaling:
                    reward = reward_scaling(reward)

                mini_states.append(state)
                mini_actions.append(action)
                if train_rlhf:
                    mini_rewards.append(reward if done else predict_reward)
                else:
                    mini_rewards.append(reward)
                mini_next_states.append(next_state)
                mini_dones.append(0 if done else 1)
                mini_logprobs.append(logprob)
                mini_values.append(value)

                writer.add_scalar('Agent/Predict reward', predict_reward, global_step=step_it)
                writer.add_scalar('Agent/Action steer mean', action_mean[0], global_step=step_it)
                writer.add_scalar('Agent/Action steer std', action_std[0], global_step=step_it)
                writer.add_scalar('Agent/Action throttle mean', action_mean[1], global_step=step_it)
                writer.add_scalar('Agent/Action throttle std', action_std[1], global_step=step_it)

                state = next_state

                if it == 60:
                    vehicle_cur_pos = [env.vehicle.get_transform().location.x, env.vehicle.get_transform().location.y]
                    dist = np.sqrt((vehicle_init_pos[0] - vehicle_cur_pos[0]) ** 2 + (vehicle_init_pos[1] - vehicle_cur_pos[1]) ** 2)
                    if dist <= 0.5:
                        simulation_error = True

                if simulation_error or total_reward <= -9.99999 and it <= 5:
                    simulation_error = True
                    # step_it -= it
                    episode_idx -= 1
                    break

                if done or it == episode_length - 1:
                    states += mini_states
                    actions += mini_actions
                    rewards += mini_rewards
                    next_states += mini_next_states
                    dones += mini_dones
                    logprobs += mini_logprobs
                    values += mini_values
                    break

            # print('--------------------TRAIN-------------------')
            if not simulation_error:
                writer.add_scalar('Metrics/Reward', total_reward, global_step=step_it)
                writer.add_scalar('Metrics/Distance traveled', env.distance_traveled, global_step=step_it)
                writer.add_scalar('Metrics/Average speed', 3.6 * env.speed_accum / env.step_count, global_step=step_it)
                writer.add_scalar('Metrics/Speed over distance', 3.6 * env.speed_accum / max(env.distance_traveled, 1), global_step=step_it)
                writer.add_scalar('Metrics/Center lane deviation', env.center_lane_deviation, global_step=step_it)
                writer.add_scalar('Metrics/Average center lane deviation', env.center_lane_deviation / env.step_count,
                                  global_step=step_it)
                writer.add_scalar('Metrics/Distance over deviation', env.center_lane_deviation / max(env.distance_traveled, 1),
                                  global_step=step_it)
                if use_rlhf or train_rlhf:
                    writer.add_scalar('Metrics/Predict Reward', total_predict_reward, global_step=step_it)
                print("Episode: ", episode_idx)
                print('Terminal_reason: ', env.terminal_reason)
                print('Reward: ', total_reward)

        if max_train_reward < total_reward:
            max_train_reward = total_reward
            model.save(0)

        # if update:
        print('----------------------')
        print('-----model update-----')
        print('----------------------')
        last_value = model.policy._forward_critic(torch.tensor(state, dtype=torch.float32)).detach().item()
        model.update(states, actions, rewards, next_states, dones, logprobs, values, last_value,
                     batch_size, minibatch_size, network_update_epochs, writer, step_it, total_steps)
        time.sleep(1)

        if episode_idx % eval_interval == 0:
            eval_reward, test_it = run_eval(env, model, model_type, test_it, writer, record_to_file + '{}.avi'.format(episode_idx), is_training=False)
            writer.add_scalar('Test/Reward', eval_reward, global_step=step_it)
            print('--------------------TEST--------------------')
            print('Test/Reward: ', eval_reward)
            print('Test/Distance traveled: ', env.distance_traveled)
            print('Test/Average speed: ', 3.6 * env.speed_accum / env.step_count)
            print('Test/Center lane deviation: ', env.center_lane_deviation)
            print('Test/Average center lane deviation: ', env.center_lane_deviation / env.step_count)
            print('Test/Distance over deviation: ', env.distance_traveled / env.center_lane_deviation)
            print('--------------------TEST--------------------')
            model.save(episode_idx)

            if max_eval_reward < eval_reward:
                max_eval_reward = eval_reward
                model.save(100000)

        # RLHF训练
        if use_rlhf:
            if not save_reward_model_data:
                reward_model.update(step_it)

            # 达到测试标准后结束
            # if env.laps_completed >= 1 and eval_reward >= 3000:
            #     if use_rlhf:
            #         if not save_reward_model_data:
            #             reward_model.update(step_it)
            #     print('Training Finish.')
            #     break
            
            # new_std = min(max(0.1, action_std_init / (max(eval_reward, 1) / 100)), 1.0)
            # model.policy.reset_action_std(new_std)

        # 达到最大训练步长后结束
        if step_it > break_steps:
            if use_rlhf:
                if not save_reward_model_data:
                    reward_model.update(step_it)
            print('Training Finish.')
            break

def float_or_list(value):
    try:
        return float(value)
    except ValueError:
        return ast.literal_eval(value)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Trains a CARLA agent with PPO")

    # RLHF
    parser.add_argument("--reward_model_n", type=int, default=REWARD_MODEL_N)
    parser.add_argument("--seq_len", type=int, default=SEQ_LEN)
    parser.add_argument("--data_type", type=str, default=DATA_TYPE)
    parser.add_argument("--dataset_save", type=str, default=DATASET_SAVE)
    parser.add_argument("--save_reward_model_data", type=bool, default=SAVE_REWARD_MODEL_DATA)
    parser.add_argument("--use_rlhf", type=bool, default=RLHF)

    # VAE
    parser.add_argument("--model_type", type=str, default=MODEL_TYPE)
    parser.add_argument("--vae_model", type=str, default=VAE_MODEL)
    parser.add_argument("--checkpoint", type=int, default=CHECKPOINT)

    # PPO
    parser.add_argument("--learning_rate", type=float, default=LEARNING_RATE)
    parser.add_argument("--learning_rate_min", type=float, default=LEARNING_RATE_MIN)
    parser.add_argument("--desired_kl", type=float, default=DESIRED_KL)
    parser.add_argument("--clip_norm", type=bool, default=CLIP_NORM)
    parser.add_argument("--loss_value_norm", type=bool, default=LOSS_VALUE_NORM)
    parser.add_argument("--reward_scaling", type=bool, default=REWARD_SCALING)
    parser.add_argument("--reward_centering", type=bool, default=REWARD_CENTERING)
    parser.add_argument("--reward_normalization", type=bool, default=REWARD_NORMALIZATION)
    parser.add_argument("--grad_clip", type=bool, default=GRAD_CLIP)
    parser.add_argument("--advantage_norm", type=bool, default=ADVANTAGE_NORM)
    parser.add_argument("--layer_norm", type=bool, default=LAYER_NORM)
    parser.add_argument("--discount_factor", type=float, default=DISCOUNT_FACTOR)
    parser.add_argument("--gae_lambda", type=float, default=GAE_LAMBDA)
    parser.add_argument("--ppo_epsilon", type=float, default=PPO_EPSILON)
    parser.add_argument("--value_scale", type=float, default=VALUE_SCALE)
    parser.add_argument("--entropy_scale", type=float, default=ENTROPY_SCALE)
    parser.add_argument("--ppo_model", type=str, default=PPO_MODEL)
    parser.add_argument("--action_std_init", type=float, default=ACTION_STD_INIT)
    parser.add_argument("--state_norm", type=bool, default=STATE_NORM)
    parser.add_argument("--network_update_epochs", type=int, default=NETWORK_UPDATE_EPOCHS)
    parser.add_argument("--batch_size", type=int, default=BATCH_SIZE)
    parser.add_argument("--minibatch_size", type=int, default=MINIBATCH_SIZE)
    parser.add_argument("--action_smoothing", type=float_or_list, default=ACTION_SMOOTHING)

    # Env
    parser.add_argument("--activate_render", type=bool, default=ACTIVATE_RENDER)
    parser.add_argument("--start_carla", type=bool, default=START_CARLA)
    parser.add_argument("--synchronous", type=bool, default=SYNCHRONOUS)
    parser.add_argument("--fps", type=int, default=FPS)
    parser.add_argument("--if_ros", type=bool, default=IF_ROS)
    parser.add_argument("--Town", type=str, default=TOWN)

    # Training
    parser.add_argument("--eval_interval", type=int, default=EVAL_INTERVAL)
    parser.add_argument("--total_steps", type=int, default=TOTAL_STEPS)
    parser.add_argument("--episode_length", type=int, default=EPISODE_LENGTH)
    parser.add_argument("--record_to_file", type=str, default=RECORD_TO_FILE)
    parser.add_argument("--break_steps", type=int, default=BREAK_STEPS)

    params = vars(parser.parse_args())

    for it in range(0, 3):
        train(params, it)