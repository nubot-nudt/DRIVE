import torch
import numpy as np
from torchvision import transforms
from CarlaEnv.wrappers import vector, get_displacement_vector

from parameters import *

def preprocess_frame(frame):
    frame = frame.astype(np.float32) / 255.0
    return frame

def preprocess_frame_(frame):
    preprocess = transforms.Compose([
        transforms.ToTensor(),
    ])
    frame = preprocess(frame).unsqueeze(0)
    return frame

def create_encode_state_fn(model_type, vae, measurements_to_include):
    """
        Returns a function that encodes the current state of
        the environment into some feature vector.
    """

    # Turn into bool array for performance
    measure_flags = ["steer" in measurements_to_include,
                     "throttle" in measurements_to_include,
                     "speed" in measurements_to_include,
                     "distance_from_center" in measurements_to_include,
                     "angle_with_line" in measurements_to_include
                     ]

    def encode_state(env):
        # Encode image with VAE
        frame = preprocess_frame(env.observation)
        frame = torch.Tensor([frame]).permute(0, 3, 1, 2)
        if model_type == 'AE':
            vae_latent = vae.encode(frame)[0].detach().numpy().squeeze()
        elif model_type == 'VAE':
            mu, logvar = vae.encode(frame)
            # vae_latent = vae.reparameterize(mu, logvar)[0].detach().numpy().squeeze()
            vae_latent = mu.detach().numpy().squeeze()

        # Append measurements
        measurements = []
        if measure_flags[0]: measurements.append(env.vehicle.control.steer)
        if measure_flags[1]: measurements.append(env.vehicle.control.throttle)
        if measure_flags[2]: measurements.append(env.vehicle.get_speed())
        if measure_flags[3]: measurements.append(env.distance_from_center)
        if measure_flags[4]: measurements.append(env.angle_with_line)

        encoded_state = np.append(vae_latent, measurements)

        return encoded_state

    return encode_state

def create_encode_state_fn_tf(vae, measurements_to_include):
    """
        Returns a function that encodes the current state of
        the environment into some feature vector.
    """

    # Turn into bool array for performance
    measure_flags = ["steer" in measurements_to_include,
                     "steer_expect" in measurements_to_include,
                     "throttle" in measurements_to_include,
                     "speed" in measurements_to_include,
                     "pitch" in measurements_to_include,
                     "waypoints" in measurements_to_include,
                     ]

    def encode_state(env):
        # Append measurements
        measurements = []
        if measure_flags[0]: measurements.append(env.vehicle.control.steer)
        if measure_flags[1]: measurements.append(env.steer_expect)
        if measure_flags[2]: measurements.append(env.vehicle.control.throttle)
        if measure_flags[3]: measurements.append(env.vehicle.get_speed())
        if measure_flags[4]: measurements.append(env.vehicle.get_transform().rotation.pitch)

        if measure_flags[5]:
            waypoints = []
            for i in range(env.current_waypoint_index, env.current_waypoint_index + n_points):
                i = i % len(env.route_waypoints)
                waypoints.append(vector(env.route_waypoints[i][0].transform.location))
            vehicle_location = vector(env.vehicle.get_location())
            theta = np.deg2rad(env.vehicle.get_transform().rotation.yaw)
            relative_waypoints = np.zeros((n_points, 2))
            for i, w_location in enumerate(waypoints):
                relative_waypoints[i] = get_displacement_vector(vehicle_location, w_location, theta)[:2]
            measurements += (relative_waypoints.flatten() / n_points).tolist()

        # Encode image with VAE
        frame = preprocess_frame(env.observation)
        vae_latent = vae.encode([frame])[0]
        vae_latent = (vae_latent + 3.0) / 6.0
        encoded_state = np.append(vae_latent, measurements)

        return encoded_state

    return encode_state

def create_encode_state_fn_torch(vae, measurements_to_include):
    """
        Returns a function that encodes the current state of
        the environment into some feature vector.
    """

    # Turn into bool array for performance
    measure_flags = ["steer" in measurements_to_include,
                     "steer_expect" in measurements_to_include,
                     "throttle" in measurements_to_include,
                     "speed" in measurements_to_include,
                     "pitch" in measurements_to_include,
                     "waypoints" in measurements_to_include,
                     ]

    def encode_state(env):
        measurements = []
        if measure_flags[0]: measurements.append(env.vehicle.control.steer)
        if measure_flags[1]: measurements.append(env.steer_expect)
        if measure_flags[2]: measurements.append(env.vehicle.control.throttle)
        if measure_flags[3]: measurements.append(env.vehicle.get_speed())
        if measure_flags[4]: measurements.append(env.vehicle.get_transform().rotation.pitch)

        if measure_flags[5]:
            waypoints = []
            for i in range(env.current_waypoint_index, env.current_waypoint_index + n_points):
                i = i % len(env.route_waypoints)
                waypoints.append(vector(env.route_waypoints[i][0].transform.location))
            vehicle_location = vector(env.vehicle.get_location())
            theta = np.deg2rad(env.vehicle.get_transform().rotation.yaw)
            relative_waypoints = np.zeros((n_points, 2))
            for i, w_location in enumerate(waypoints):
                relative_waypoints[i] = get_displacement_vector(vehicle_location, w_location, theta)[:2]
            measurements += (relative_waypoints.flatten() / n_points).tolist()

        # Encode image with VAE
        frame = preprocess_frame_(env.observation)
        mu, logvar = vae.encode(frame)
        vae_latent = vae.reparameterize(mu, logvar)[0].detach().numpy().squeeze()

        encoded_state = np.append((vae_latent + 4.0) / 8.0, measurements)

        # measurements = []
        # if measure_flags[0]: measurements.append(env.vehicle.control.steer)
        # if measure_flags[1]: measurements.append(env.steer_expect)
        # if measure_flags[2]: measurements.append(env.vehicle.control.throttle)
        # if measure_flags[3]: measurements.append(env.vehicle.get_speed())
        #
        # if measure_flags[4]:
        #     waypoints = []
        #     for i in range(env.current_waypoint_index, env.current_waypoint_index + 15):
        #         i = i % len(env.route_waypoints)
        #         waypoints.append(vector(env.route_waypoints[i][0].transform.location))
        #     vehicle_location = vector(env.vehicle.get_location())
        #     theta = np.deg2rad(env.vehicle.get_transform().rotation.yaw)
        #     relative_waypoints = np.zeros((15, 2))
        #     for i, w_location in enumerate(waypoints):
        #         relative_waypoints[i] = get_displacement_vector(vehicle_location, w_location, theta)[:2]
        #     measurements += relative_waypoints.flatten().tolist()
        #
        # # Encode image with VAE
        # frame = preprocess_frame_(env.observation)
        # mu, logvar = vae.encode(frame)
        # vae_latent = vae.reparameterize(mu, logvar)[0].detach().numpy().squeeze()
        #
        # encoded_state = np.append(vae_latent, measurements)

        return encoded_state

    return encode_state



class RunningMeanStd:
    # Dynamically calculate mean and std
    def __init__(self, shape):  # shape:the dimension of input data
        self.n = 0
        self.mean = np.zeros(shape)
        self.S = np.zeros(shape)
        self.std = np.sqrt(self.S)

    def update(self, x):
        x = np.array(x)
        self.n += 1
        if self.n == 1:
            self.mean = x
            self.std = x
        else:
            old_mean = self.mean.copy()
            self.mean = old_mean + (x - old_mean) / self.n
            self.S = self.S + (x - old_mean) * (x - self.mean)
            self.std = np.sqrt(self.S / self.n)

class Normalization:
    def __init__(self, shape):
        self.running_ms = RunningMeanStd(shape=shape)

    def __call__(self, x, update=True):
        # Whether to update the mean and std,during the evaluating,update=False
        if update:
            self.running_ms.update(x)
        x = (x - self.running_ms.mean) / (self.running_ms.std + 1e-8)

        return x

class RewardScaling:
    def __init__(self, shape, gamma):
        self.shape = shape  # reward shape=1
        self.gamma = gamma  # discount factor
        self.running_ms = RunningMeanStd(shape=self.shape)
        self.R = np.zeros(self.shape)

    def __call__(self, x):
        self.R = self.gamma * self.R + x
        self.running_ms.update(self.R)
        x = x / (self.running_ms.std + 1e-8)  # Only divided std
        return x

    def reset(self):  # When an episode is done,we should reset 'self.R'
        self.R = np.zeros(self.shape)