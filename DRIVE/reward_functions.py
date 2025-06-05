import numpy as np
from parameters import *
from CarlaEnv.wrappers import angle_diff, vector

def create_reward_fn(reward_fn, max_speed=-1):
    """
        Wraps input reward function in a function that adds the
        custom termination logic used in these experiments

        reward_fn (function(CarlaEnv)):
            A function that calculates the agent's reward given
            the current state of the environment.
        max_speed:
            Optional termination criteria that will terminate the
            agent when it surpasses this speed.
            (If training with reward_kendal, set this to 20)
    """
    def func(env):
        terminal_reason = "Running..."

        # Stop if speed is less than 1.0 km/h after the first 5s of an episode
        env.low_speed_timer += 1.0 / env.fps # 5秒后开始low_speed_timer > 5.0，因为一帧30秒
        speed = env.vehicle.get_speed()
        if env.low_speed_timer > 5.0 and speed < 1.0 / 3.6: # 说明车辆发生碰撞或者仿真异常
            env.terminal_state = True
            terminal_reason = "Vehicle stopped"

        # Stop if distance from center > max distance
        if abs(env.distance_from_center) > max_distance: # or abs(env.angle_with_line) > max_angle:
            env.terminal_state = True
            terminal_reason = "Off-track"

        # Stop if speed is too high
        if max_speed > 0 and speed_kmh > max_speed:
            env.terminal_state = True
            terminal_reason = "Too fast"

        # Calculate reward
        reward = 0
        if env.terminal_state:
            reward -= 10
            env.extra_info.extend([
                env.terminal_reason,
                ""
            ])
            print('terminal_reason: ', env.terminal_reason)
        else:
            reward += reward_fn(env)

        return reward
    return func

def reward_speed_centering_angle_multiply(env):
    """
        reward = Positive speed reward for being close to target speed,
                 however, quick decline in reward beyond target speed
               * centering factor (1 when centered, 0 when not)
               * angle factor (1 when aligned with the road, 0 when more than 20 degress off)
    """

    angle = env.vehicle.get_angle(env.current_waypoint)
    speed_kmh = 3.6 * env.vehicle.get_speed()
    # if speed_kmh < min_speed:  # When speed is in [0, min_speed] range
    #     speed_reward = speed_kmh / min_speed  # Linearly interpolate [0, 1] over [0, min_speed]
    # elif speed_kmh > target_speed:  # When speed is in [target_speed, inf]
    #     # Interpolate from [1, 0, -inf] over [target_speed, max_speed, inf]
    #     speed_reward = 1.0 - (speed_kmh - target_speed) / (max_speed - target_speed)
    # else:  # Otherwise
    #     speed_reward = 1.0  # Return 1 for speeds in range [min_speed, target_speed]
    speed_reward = np.exp(-((speed_kmh - target_speed) ** 2) / 2)

    # Interpolated from 1 when centered to 0 when 3 m from center
    centering_factor = max(1.0 - env.distance_from_center / max_distance, 0.0)

    # Interpolated from 1 when aligned with the road to 0 when +/- 20 degress of road
    angle_factor = max(1.0 - abs(angle / np.deg2rad(max_angle_center_lane)), 0.0)

    std = np.std(env.distance_from_center_history)
    distance_std_factor = max(1.0 - abs(std / max_std_center_lane), 0.0)

    # Final reward
    reward = speed_reward * centering_factor * angle_factor * distance_std_factor
    # reward = speed_reward + centering_factor # + angle_factor + distance_std_factor

    return reward

reward_func = create_reward_fn(reward_speed_centering_angle_multiply)