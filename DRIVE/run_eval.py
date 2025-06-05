import os
import ast
import torch
import random
import argparse
import numpy as np
import tensorflow as tf

from ppo import PPO
from parameters import *
from vae.models import VAE, AE, VAE_torch
from utils import VideoRecorder
from vae.models_tf import ConvVAE
from reward_functions import reward_func
from vae_common import create_encode_state_fn_tf, create_encode_state_fn, create_encode_state_fn_torch

USE_ROUTE_ENVIRONMENT = False
if USE_ROUTE_ENVIRONMENT:
    from CarlaEnv.carla_route_env import CarlaRouteEnv as CarlaEnv
else:
    from CarlaEnv.carla_lap_env import CarlaLapEnv as CarlaEnv

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

def run_eval(env, model, model_type, test_it=0, writer=None, video_filename=None, is_training=False, state_norm=None):
    # Init test env
    rendered_frame = env.render(mode="rgb_array")

    # Init video recording
    if video_filename is not None:
        print("Recording video to {} ({}x{}x{}@{}fps)".format(video_filename, *rendered_frame.shape,
                                                              int(env.average_fps)))
        video_recorder = VideoRecorder(video_filename,
                                       frame_size=rendered_frame.shape,
                                       fps=env.average_fps)
        video_recorder.add_frame(rendered_frame)
    else:
        video_recorder = None

    model.eval()
    # While non-terminal state
    state, terminal, total_reward = env.reset(is_training=is_training), False, 0.0
    while not terminal:
        test_it += 1
        env.extra_info.append("Running eval...")
        env.extra_info.append("")

        # Take deterministic actions at test time (std=0)
        if state_norm:
            state = state_norm(state)

        action = model.action_inference(state)
        if writer:
            writer.add_scalar('Test/Steer', action[0], global_step=test_it)
            writer.add_scalar('Test/Throttle', action[1], global_step=test_it)
        state, reward, terminal, predict_reward = env.step(action)

        if terminal:
            break

        # Add frame
        rendered_frame = env.render(mode="rgb_array")
        if video_recorder is not None:
            video_recorder.add_frame(rendered_frame)
        total_reward += reward

    # Release video
    if video_recorder is not None:
        video_recorder.release()

    model.train()
    return total_reward, test_it

def float_or_list(value):
    try:
        return float(value)
    except ValueError:
        return ast.literal_eval(value)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Trains a CARLA agent with PPO")

    parser.add_argument("--ppo_model", type=str, default=PPO_MODEL)

    parser.add_argument("--model_type", type=str, default=MODEL_TYPE)
    parser.add_argument("--vae_model", type=str, default=VAE_MODEL)
    parser.add_argument("--checkpoint", type=int, default=CHECKPOINT)

    parser.add_argument("--start_carla", type=bool, default=START_CARLA)
    parser.add_argument("--fps", type=int, default=FPS)
    parser.add_argument("--action_smoothing", type=float_or_list, default=ACTION_SMOOTHING)
    parser.add_argument("--record_to_file", type=str, default=RECORD_TO_FILE)
    parser.add_argument("--if_ros", type=bool, default=IF_ROS)
    parser.add_argument("--activate_render", type=bool, default=ACTIVATE_RENDER)
    parser.add_argument("--synchronous", type=bool, default=SYNCHRONOUS)
    parser.add_argument("--Town", type=str, default=TOWN)

    params = vars(parser.parse_args())

    if True:
        ppo_model = params["ppo_model"]

        model_type = params["model_type"]
        vae_model = params["vae_model"]
        checkpoint = params["checkpoint"]

        start_carla = params["start_carla"]
        fps = params["fps"]
        action_smoothing = params["action_smoothing"]
        record_to_file = params["record_to_file"]
        if_ros = params["if_ros"]
        activate_render = params["activate_render"]
        synchronous = params["synchronous"]
        Town = params["Town"]

        if not os.path.exists(record_to_file):
            os.makedirs(record_to_file)

    # Load VAE
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

    # Create env
    print("Creating environment...")
    env = CarlaEnv(obs_res=(160, 80),
                   action_smoothing=action_smoothing,
                   encode_state_fn=encode_state_func,
                   reward_fn=reward_func,
                   fps=fps,
                   activate_render=activate_render,
                   start_carla=start_carla,
                   synchronous=synchronous,
                   vae=vae,
                   if_ros=if_ros,
                   Town=Town,
                   )

    # # Set seeds
    # seed = 0
    # if isinstance(seed, int):
    #     tf.random.set_random_seed(seed)
    #     np.random.seed(seed)
    #     random.seed(seed)
    #     env.seed(seed)

    # Create model
    print("Creating model...")
    input_shape = Z_DIM + len(measurements_to_include) + 29
    model = PPO(input_shape, env.action_space,
                ppo_model=ppo_model + 'models/')  # PPO初始化
    model.load(515)

    # Run eval
    print("Running eval...")
    run_eval(env, model, model_type, video_filename=record_to_file + 'eval.avi')

    # Close env
    print("Done!")
    env.close()