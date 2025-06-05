import os
import cv2
import csv
import gym
import time
import torch
import carla
import random
import pygame
import subprocess
from pygame.locals import *
from collections import deque
import rospy
from std_msgs.msg import Float64, Int8, Float64MultiArray

from hud import HUD
from wrappers import *
from planner import RoadOption, compute_route_waypoints

import sys
sys.path.append('/home/jk/Coppeliasim_carla/carla_0_9_5/PythonAPI/carla/dist/carla-0.9.5-py3.5-linux-x86_64.egg')

class CarlaLapEnv(gym.Env):

    def __init__(self, 
                 host="127.0.0.1", 
                 port=2000,
                 viewer_res=(1280, 720), 
                 obs_res=(1280, 720),
                 fps=30, 
                 activate_render=True, 
                 start_carla=True,
                 is_training=True, 
                 synchronous=False,
                 if_ros=False, 
                 Town='Town02',

                 vae=None, 
                 
                 action_smoothing=0.9,
                 reward_fn=None, 
                 encode_state_fn=None, 
                 
                 writer=None,
                 
                 use_rlhf=False, 
                 state_dim=1, 
                 seq_len=30, 
                 data_type='image',
                 save_reward_model_data=False,
                 it = 0,
                 ):

        # 启动CARLA
        self.carla_process = None
        if start_carla:
            if "CARLA_ROOT" not in os.environ:
                raise Exception("${CARLA_ROOT} has not been set!")
            carla_path = os.path.join(os.environ["CARLA_ROOT"], "CarlaUE4.sh")
            launch_command = [carla_path]
            launch_command += ['-windowed']
            launch_command += ["-ResX=128"]
            launch_command += ["-ResY=64"]
            launch_command += ['-quality_level=Low']
            if synchronous:
                launch_command += ['-benchmark']
                launch_command += ["-fps=%i" % fps]
            # launch_command += ['-RenderOffScreen']
            launch_command += ['-prefernvidia']
            print("Running command:")
            print(" ".join(launch_command))
            self.carla_process = subprocess.Popen(launch_command, stdout=subprocess.DEVNULL)
            print("Waiting for CARLA to initialize")
            # ./CarlaUE4.sh -quality_level=Low -benchmark -fps=15 -RenderOffScreen
            time.sleep(5)

        # 窗口分辨率
        width, height = viewer_res  # 可视化窗口分辨率
        if obs_res is None:
            out_width, out_height = width, height
        else:
            out_width, out_height = obs_res # 图像分辨率

        # 传参
        self.action_space = gym.spaces.Box(np.array([-1, 0]), np.array([1, 1]), dtype=np.float32)  # steer, throttle # 动作最值
        self.observation_space = gym.spaces.Box(low=0.0, high=1.0, shape=(*obs_res, 3), dtype=np.float32)  # 状态最值
        self.average_fps = self.fps = fps  # 帧率30
        self.fps_inv = 1.0 / self.fps
        self.activate_render = activate_render
        self.is_training = is_training
        self.synchronous = synchronous
        self.if_ros = if_ros
        self.Town = Town

        self.vae = vae

        self.action_smoothing = action_smoothing  # 动作平滑度，给的0.0
        self.encode_state_fn = (lambda x: x) if not callable(encode_state_fn) else encode_state_fn  # VAE编码器
        self.reward_fn = (lambda x: 0) if not callable(reward_fn) else reward_fn  # 奖励函数

        self.writer = writer

        self.use_rlhf = use_rlhf
        self.state_dim = state_dim
        self.reward_model = None
        self.seq_len = seq_len
        self.data_type = data_type
        self.save_reward_model_data = save_reward_model_data

        self.step_it = 0

        # 与carla通信
        self.world = None
        try:
            # Connect to carla
            self.client = carla.Client(host, port) # 启动客户端
            self.client.set_timeout(60.0)
            if it == 0:
                if Town == 'Town02':
                    self.client.load_world('/Game/Carla/Maps/Town02')  # 加载地图
                else:
                    self.client.load_world('/Game/Carla/Maps/Town07') # 加载地图

            # Create world wrapper
            self.world = World(self.client)
            if synchronous:
                settings = self.world.get_settings()
                settings.fixed_delta_seconds = 1 / self.fps
                settings.synchronous_mode = synchronous # 开启同步模式
                self.world.apply_settings(settings)

            # 获取地图起始位置
            if Town == 'Town02':
                start_idx = 73
            else:
                start_idx = 76 # 76 for Town07, 73 for Town02
            lap_start_wp = self.world.map.get_waypoint(self.world.map.get_spawn_points()[start_idx].location) # 获取给定位置location对应的最近路径点waypoint。路径点是 Carla 中用于表示道路网络的关键点，定义了车辆可以行驶的路径。
            spawn_transform = lap_start_wp.transform # 车辆起始位置和方向
            spawn_transform.location += carla.Location(z=1) # 设置Z值

            # Create vehicle and attach camera to it
            self.vehicle = Vehicle(self.world, spawn_transform,
                                   on_collision_fn=lambda e: self._on_collision(e),
                                   on_invasion_fn=lambda e: self._on_invasion(e)) # 车辆初始化以及碰撞传感器、车道传感器初始化

            # 训练可视化窗口
            if self.activate_render:
                pygame.init()
                pygame.font.init()
                self.display = pygame.display.set_mode((width, height), pygame.HWSURFACE | pygame.DOUBLEBUF)
                self.clock = pygame.time.Clock()
                self.hud = HUD(width, height)
                self.hud.set_vehicle(self.vehicle)
                self.world.on_tick(self.hud.on_world_tick)

            # Create cameras
            self.dashcam = Camera(self.world, out_width, out_height,
                                  transform=camera_transforms["dashboard"],
                                  attach_to=self.vehicle, on_recv_image=lambda e: self._set_observation_image(e),
                                  sensor_tick=0.0 if synchronous else 1.0/self.fps) # 状态观测图像即是这个
            self.camera  = Camera(self.world, width, height,
                                  transform=camera_transforms["spectator"],
                                  attach_to=self.vehicle, on_recv_image=lambda e: self._set_viewer_image(e),
                                  sensor_tick=0.0 if synchronous else 1.0/self.fps) # 第三视角
        except Exception as e:
            self.close()
            raise e

        # Generate waypoints along the lap
        if Town == 'Town02':
            plan_ = [RoadOption.RIGHT] + [RoadOption.LEFT] + [RoadOption.RIGHT] + [RoadOption.LEFT] + [RoadOption.STRAIGHT] + [RoadOption.LEFT]  # for Town02
        else:
            plan_ = [RoadOption.STRAIGHT] + [RoadOption.RIGHT] * 2 + [RoadOption.STRAIGHT] * 5 # for Town07
        self.route_waypoints = compute_route_waypoints(self.world.map, lap_start_wp, lap_start_wp, resolution=1.0,
                                                       plan=plan_) # 生成LAP环境下的路径点，起点和终点是同一位置，即车辆绕圈跑。每两个点之间的间隔至少1米
        self.current_waypoint_index = 0 # 当前路径点索引
        self.checkpoint_waypoint_index = 0 # 车辆重置位置刷新

        # for debug
        self.debug_point = self.route_waypoints[0][0].transform.location

        if self.if_ros:
            # ROS接入Coppeliasim
            rospy.init_node('carla_lap_env', anonymous=True)
            self.rate = rospy.Rate(self.fps)
            # self.wheel_angle_sub = rospy.Subscriber('/wheel_angle_actual', Float64, self.wheel_angle_callback)
            self.wheel_angle_sub = rospy.Subscriber('/end_pos_fb_l', Float64MultiArray, self.wheel_angle_callback)
            self.wheel_angle_expect_pub = rospy.Publisher('/wheel_angle_expect', Float64, queue_size=1)
            self.wheel_angle_expect = Float64()
            self.wheel_angle_expect.data = 0.0
            self.wheel_margin = 0.95 # 方向盘裕度，实际上每次Coppeliasim重启会发现方向盘最小最大位置都不定
            self.wheel_angle_zero = -1.20966 # 方向盘初始，默认这个值开始为0，若处于该值与min之间，则左转，否则右转
            self.wheel_left = (self.wheel_angle_zero - (-2.191557845734586)) * self.wheel_margin
            self.wheel_right = (-0.05785066388564397 - self.wheel_angle_zero) * self.wheel_margin
            self.wheel_angle_actual = self.wheel_angle_zero
            self.wheel_vel = (1.0 - (-1.0)) / (3.0 + 0.2) # 给个0.2秒裕度
            self.coppeliasim_reset_pub = rospy.Publisher('/restart_sim', Int8, queue_size=1)
            self.coppeliasim_reset_sub = rospy.Subscriber('/restart_status', Int8, self.coppeliasim_reset_callback)
            self.coppeliasim_reset = Int8()
            self.coppeliasim_reset.data = 0

        # 环境初始化
        self.reset()

    def reward_model_init(self, reward_model):
        self.reward_model = reward_model

    def reward_model_reset(self):
        if self.save_reward_model_data and self.reward_model:
            self.reward_model.reset_temp_experience()

        self.seq_obs = deque(maxlen=self.seq_len)
        self.seq_actions = deque(maxlen=self.seq_len)
        # 协助人类打标签参数
        self.seq_reward = deque(maxlen=self.seq_len)
        self.seq_distance_from_center = deque(maxlen=self.seq_len)
        self.seq_angle_from_center = deque(maxlen=self.seq_len)
        self.seq_road_maneuver = "Void"
        self.seq_viewer_image = deque(maxlen=self.seq_len)

    def reset(self, is_training=True):
        if self.if_ros:
            # Coppeliasim环境初始化
            self.coppeliasim_reset.data = 1
            while self.coppeliasim_reset.data:
                self.coppeliasim_reset_pub.publish(self.coppeliasim_reset)
                time.sleep(1)

        # 重置车辆控制量
        self.vehicle.control.steer = float(0.0) # 初始前轮转角
        self.vehicle.control.throttle = float(0.0) # 初始油门
        #self.vehicle.control.brake = float(0.0) # 初始刹车
        # self.vehicle.tick() # 控制生效
        self.vehicle.set_simulate_physics(False) # Reset the car's physics

        # if self.synchronous:
        #     ticks = 0
        #     while ticks < self.fps * 2:
        #         self.world.tick()
        #         try:
        #             self.world.wait_for_tick(seconds=1.0 / self.fps + 0.1)
        #             ticks += 1
        #         except:
        #             pass
        # else:
        #     time.sleep(0.2)

        # 重置车辆起始状态
        if is_training:
            # 将车辆位置随机重置
            self.checkpoint_waypoint_index = random.randint(0, len(self.route_waypoints) - 1)
            waypoint, _ = self.route_waypoints[self.checkpoint_waypoint_index]
            self.current_waypoint_index = self.checkpoint_waypoint_index # 更新ID
        else:
            # 评估就从原点开始
            waypoint, _ = self.route_waypoints[0]
            self.current_waypoint_index = 0
        if self.Town == 'Town07':
            waypoint.transform.location += carla.Location(z=1) # 设置Z值
        self.vehicle.set_transform(waypoint.transform) # 设置车辆位置
        self.vehicle.collision_sensor.collision_data.clear() # 清空车辆碰撞情况

        # RL状态初始化/指标
        self.closed = False
        self.terminal_state = False # 该回合结束FLAG
        self.terminal_reason = None # 终止原因
        self.observation = self.observation_buffer = None # 状态量
        self.viewer_image = self.viewer_image_buffer = None # 第三视角图像
        self.step_count = 0 # 计步器
        self.speed_accum = 0.0 # 累积加速度
        self.total_reward = 0.0
        self.laps_completed = 0.0 # 跑圈完成率
        self.low_speed_timer = 0.0 # 回合终止条件
        self.angle_with_line = 0.0 # 车辆与道路的夹角，左负右正
        self.distance_traveled = 0.0 # 行驶里程
        self.center_lane_deviation = 0.0 # 累积偏离道路距离
        self.extra_info = [] # 交互界面上的额外信息
        initial_values = [0] * 30
        self.distance_from_center_history = deque(initial_values, maxlen=30) # 记录连续30个点与道路中心的距离
        self.start_waypoint_index = self.current_waypoint_index # 开始点
        self.previous_location = self.vehicle.get_transform().location # 车辆上一时刻位置

        # 修改
        self.steer_expect = 0.0
        self.steer_actual = 0.0
        self.last_steer_actual = 0.0
        self.last_steer_expect = 0.0

        # DEBUG: Draw path
        # self._draw_path(life_time=1000.0, skip=1)

        if self.use_rlhf:
            self.reward_model_reset()

        self.vehicle.set_simulate_physics(True)  # 设置车辆物理属性
        self.world.tick()
        time.sleep(0.2)
        obs = self.step(None)[0]
        time.sleep(0.2)

        if self.use_rlhf:
            self.seq_obs.append(obs)

        # Return initial observation
        return obs

    def close(self):
        if self.carla_process:
            self.carla_process.terminate()
        pygame.quit()
        if self.world is not None:
            self.world.destroy()
        self.closed = True

    def render(self, mode="human"): # 渲染
        if mode == "rgb_array_no_hud":
            return self.viewer_image
        elif mode == "rgb_array":
            # Turn display surface into rgb_array
            return np.array(pygame.surfarray.array3d(self.display), dtype=np.uint8).transpose([1, 0, 2])
        elif mode == "state_pixels":
            return self.observation
        
        # Tick render clock
        self.clock.tick() # 交互界面更新
        self.hud.tick(self.world, self.clock, steer_expect_ = self.steer_expect)

        # Get maneuver name
        if self.current_road_maneuver == RoadOption.LANEFOLLOW: maneuver = "Follow Lane"
        elif self.current_road_maneuver == RoadOption.LEFT:     maneuver = "Left"
        elif self.current_road_maneuver == RoadOption.RIGHT:    maneuver = "Right"
        elif self.current_road_maneuver == RoadOption.STRAIGHT: maneuver = "Straight"
        elif self.current_road_maneuver == RoadOption.VOID:     maneuver = "VOID"
        else:                                                   maneuver = "INVALID(%i)" % self.current_road_maneuver

        # Add metrics to HUD
        self.extra_info.extend([
            "Reward: % 19.2f" % self.last_reward,
            "",
            "Maneuver:        % 11s"       % maneuver,
            "Laps completed:    % 7.2f %%" % (self.laps_completed * 100.0),
            "Distance traveled: % 7d m"    % self.distance_traveled,
            "Center deviance:   % 7.2f m"  % self.distance_from_center,
            "Angle deviance:  % 7.2f rad"  % self.angle_with_line,
            "Avg center dev:    % 7.2f m"  % (self.center_lane_deviation / self.step_count),
            "Avg speed:      % 7.2f km/h"  % (3.6 * self.speed_accum / self.step_count),
            "Total reward:        % 7.2f"  % self.total_reward,
        ])

        # Blit image from spectator camera
        self.display.blit(pygame.surfarray.make_surface(self.viewer_image.swapaxes(0, 1)), (0, 0))

        # # 相机视角
        # # Superimpose current observation into top-right corner
        # obs_h, obs_w = self.observation.shape[:2]
        # view_h, view_w = self.viewer_image.shape[:2]
        # pos = (view_w - obs_w - 10, 10)
        # self.display.blit(pygame.surfarray.make_surface(self.observation.swapaxes(0, 1)), pos)
        #
        # # VAE重建相机视角
        # pos_ = (view_w - obs_w - 10, 20 + obs_h)
        # # torch visualization
        # decode_image, _, _ = self.vae(torch.Tensor([self.observation / 255]).permute(0, 3, 1, 2))
        # decode_image = decode_image.detach().squeeze().numpy().T
        # # # tensorflow visualization
        # # decode_image = self.vae.generate_from_latent(self.vae.encode([self.observation / 255])).reshape(80, 160).T
        # self.display.blit(pygame.surfarray.make_surface(decode_image), pos_)

        # Render HUD
        self.hud.render(self.display, extra_info=self.extra_info)
        self.extra_info = [] # Reset extra info list

        # Render to screen
        pygame.display.flip()

    def step(self, action, it = 0):
        if self.closed:
            raise Exception("CarlaEnv.step() called after the environment was closed." +
                            "Check for info[\"closed\"] == True in the learning loop.")

        if not self.synchronous:
            if self.fps <= 0:
                # Go as fast as possible
                self.clock.tick()
            else:
                # Sleep to keep a steady fps
                self.clock.tick_busy_loop(self.fps)

            # Update average fps (for saving recordings)
            # if action is not None:
            #     self.average_fps = self.average_fps * 0.5 + self.clock.get_fps() * 0.5

        # 采取动作，考虑动作平滑度
        if action is not None:
            action = self.action_space.low + (action + 1.0) * 0.5 * (self.action_space.high - self.action_space.low)
            action = np.clip(action, self.action_space.low, self.action_space.high)

            if self.if_ros:
                self.steer_expect, throttle = [float(a) for a in action]
                if isinstance(self.action_smoothing, list):  # 动作平滑
                    self.steer_expect = self.last_steer_expect * self.action_smoothing[0] + self.steer_expect * (1.0 - self.action_smoothing[0])
                    self.vehicle.control.throttle = self.vehicle.control.throttle * self.action_smoothing[1] + throttle * (1.0 - self.action_smoothing[1])
                else:
                    self.steer_expect = self.last_steer_expect * self.action_smoothing + self.steer_expect * (1.0 - self.action_smoothing)
                    self.vehicle.control.throttle = self.vehicle.control.throttle * self.action_smoothing + throttle * (1.0 - self.action_smoothing)

                # 与Coppeliasim联调
                self.wheel_angle_expect.data = self.steer_expect  # 发给Coppeliasim期望转向
                self.wheel_angle_expect_pub.publish(self.wheel_angle_expect)
                self.steer_actual = self.wheel_angle_actual  # 从Coppeliasim订阅实际转向

                self.vehicle.control.steer = self.steer_actual
            else:
                self.steer_actual, throttle = [float(a) for a in action] # 读取转角和油门
                if isinstance(self.action_smoothing, list): # 动作平滑
                    self.vehicle.control.steer = self.vehicle.control.steer * self.action_smoothing[0] + self.steer_actual * (1.0 - self.action_smoothing[0])
                    self.vehicle.control.throttle = self.vehicle.control.throttle * self.action_smoothing[1] + throttle * (1.0 - self.action_smoothing[1])
                else:
                    self.vehicle.control.steer    = self.vehicle.control.steer * self.action_smoothing + self.steer_actual * (1.0-self.action_smoothing)
                    self.vehicle.control.throttle = self.vehicle.control.throttle * self.action_smoothing + throttle * (1.0-self.action_smoothing)

            # 日志记录
            if self.writer != None:
                self.writer.add_scalar('Agent/Steer actual', self.steer_actual, self.step_it)
                self.writer.add_scalar('Agent/Throttle', throttle, self.step_it)
                self.writer.add_scalar('Agent/Speed', self.vehicle.get_speed(), self.step_it)
                if self.if_ros:
                    self.writer.add_scalar('Agent/Steer expect', self.steer_expect, self.step_it)
                    self.writer.add_scalar('Agent/Steer diff,', self.last_steer_expect - self.steer_actual, self.step_it)
                self.step_it += 1

            if self.use_rlhf:
                self.seq_actions.append([self.vehicle.control.steer, self.vehicle.control.throttle])

        # Tick game
        self.world.tick()

        # if self.synchronous:
        #     self.clock.tick()
        #     while True:
        #         try:
        #             self.world.wait_for_tick(seconds=1.0 / self.fps + 0.1)
        #             break
        #         except:
        #             # Timeouts happen occasionally for some reason, however, they seem to be fine to ignore
        #             self.world.tick()

        # 获取车辆位姿
        transform = self.vehicle.get_transform()

        # 保持跟踪最近点
        waypoint_index = self.current_waypoint_index # 当前路径点ID
        for _ in range(len(self.route_waypoints)): # 更新ID并判断是否还需要继续更新，即每次更新一个路径点
            # Check if we passed the next waypoint along the route
            next_waypoint_index = waypoint_index + 1 # 更新ID
            wp, _ = self.route_waypoints[next_waypoint_index % len(self.route_waypoints)]
            dot = np.dot(vector(wp.transform.get_forward_vector())[:2],
                         vector(transform.location - wp.transform.location)[:2])
            if dot > 0.0: # Did we pass the waypoint?
                waypoint_index += 1 # Go to next waypoint
            else:
                break
        self.current_waypoint_index = waypoint_index

        # 计算偏离路径距离
        self.current_waypoint, self.current_road_maneuver = self.route_waypoints[self.current_waypoint_index    % len(self.route_waypoints)]
        self.next_waypoint, self.next_road_maneuver       = self.route_waypoints[(self.current_waypoint_index+1) % len(self.route_waypoints)]
        self.distance_from_center = distance_to_line(vector(self.current_waypoint.transform.location),
                                                     vector(self.next_waypoint.transform.location),
                                                     vector(transform.location))
        self.center_lane_deviation += self.distance_from_center

        # 计算总行驶里程
        if it == 1:
            self.distance_traveled = 0
        else:
            self.distance_traveled += self.previous_location.distance(transform.location)
        self.previous_location = transform.location

        # 累积速度
        self.speed_accum += self.vehicle.get_speed()
        
        # 计算跑圈完成率
        self.laps_completed = (self.current_waypoint_index - self.start_waypoint_index) / len(self.route_waypoints) # 计算跑圈完成率
        if self.laps_completed >= 1:
            self.terminal_state = True # 跑完圈即结束

        self.distance_from_center_history.append(self.distance_from_center)

        # 奖励值
        # 计算当前帧奖励
        self.last_reward = self.reward_fn(self)

        # 修改奖励在此处：
        # rew_steer_expect_diff = (self.last_steer_actual - self.steer_actual) ** 2
        # if self.writer != None:
        #     self.writer.add_scalar("Agent/Rew origin", self.last_reward, global_step=self.step_it)
        #     self.writer.add_scalar("Agent/Rew steer expect", rew_steer_expect_diff, global_step=self.step_it)
        self.last_steer_actual = self.steer_actual
        self.last_steer_expect = self.steer_expect
        #
        # self.last_reward -= 0.1 * rew_steer_expect_diff

        self.total_reward += self.last_reward
        # 计步+1
        self.step_count += 1

        # 状态量
        # Get most recent observation and viewer image
        self.observation = self._get_observation() # 获取图像
        self.viewer_image = self._get_viewer_image() # 获取第三视角图像
        encoded_state = self.encode_state_fn(self) # 图像编码

        # RLHF
        predict_reward = 0
        if action is not None:
            if self.use_rlhf:
                self.seq_distance_from_center.append(self.distance_from_center)
                self.seq_angle_from_center.append(self.vehicle.get_angle(self.current_waypoint))
                self.seq_reward.append(self.last_reward)
                if self.data_type == 'image':
                    self.seq_viewer_image.append(cv2.resize(self.viewer_image, (640, 360)))

                maneuver = "Void"
                if self.current_road_maneuver == RoadOption.LANEFOLLOW:
                    maneuver = "Follow Lane"
                elif self.current_road_maneuver == RoadOption.LEFT:
                    maneuver = "Left"
                elif self.current_road_maneuver == RoadOption.RIGHT:
                    maneuver = "Right"
                elif self.current_road_maneuver == RoadOption.STRAIGHT:
                    maneuver = "Straight"
                self.seq_road_maneuver = maneuver
                # 计算预测奖励
                seq_obs = torch.tensor(self.seq_obs, dtype=torch.float32)
                seq_actions = torch.tensor(self.seq_actions, dtype=torch.float32)
                # predict_reward = self.reward_model.predict(seq_obs, seq_actions)
                preds, rho = self.reward_model.predict(seq_obs, seq_actions)
                if abs(self.vehicle.get_speed() * 3.6 - 12) > 3.0:
                    preds[3:] = 0.0
                pred_reward = torch.mean(preds, dim=0)
                pred_reward_std = torch.std(preds, dim = 0)
                predict_reward = pred_reward[-1] + pred_reward_std[-1] * rho

                # predict_reward = self.reward_model.predict(torch.tensor(self.distance_from_center, dtype=torch.float32))
                if it > 5: # and it % (self.seq_len if self.seq_len > 1 else 30) == 0: # 让数据有区分度，而不是时时刻刻存
                    self.reward_model.add_temp_experience(self.seq_obs, self.seq_actions, self.seq_reward, self.seq_distance_from_center, self.seq_angle_from_center, self.seq_road_maneuver, self.seq_viewer_image)
                # 存储新的状态
                self.seq_obs.append(encoded_state)

        # Check for ESC press
        if self.activate_render:
            pygame.event.pump()
            if pygame.key.get_pressed()[K_ESCAPE]:
                self.close()
                self.terminal_state = True
            self.render()

        if self.if_ros:
            self.rate.sleep()

        return encoded_state, self.last_reward, self.terminal_state, predict_reward

    def _get_observation(self):
        while self.observation_buffer is None:
            pass
        obs = self.observation_buffer.copy()
        self.observation_buffer = None
        return obs

    def _get_viewer_image(self):
        while self.viewer_image_buffer is None:
            pass
        image = self.viewer_image_buffer.copy()
        self.viewer_image_buffer = None
        return image

    def _on_collision(self, event):
        # 与非道路发生碰撞即结束
        # if get_actor_display_name(event.other_actor) != "Road":
        #     self.terminal_state = True
        #     self.terminal_reason = "escape from road."
        if self.activate_render:
            self.hud.notification("Collision with {}".format(get_actor_display_name(event.other_actor)))

    def _on_invasion(self, event):
        lane_types = set(x.type for x in event.crossed_lane_markings)
        text = ["%r" % str(x).split()[-1] for x in lane_types]
        if self.activate_render:
            self.hud.notification("Crossed line %s" % " and ".join(text))

    def _set_observation_image(self, image):
        self.observation_buffer = image

    def _set_viewer_image(self, image):
        self.viewer_image_buffer = image

    def wheel_angle_callback(self, data):
        # [self.wheel_angle_zero, self.wheel_angle_min] -> [0, 1]
        # [self.wheel_angle_zero, self.wheel_angle_max] -> [0, -1]
        temp_var = data.data[-1] - self.wheel_angle_zero
        if temp_var <= 0:  # 左转
            self.wheel_angle_actual = temp_var / self.wheel_left
        else:  # 右转
            self.wheel_angle_actual = temp_var / self.wheel_right

    def coppeliasim_reset_callback(self, data):
        self.coppeliasim_reset.data = data.data
        print('Coppeliasim restart.')

    def _draw_path(self, life_time=60.0, skip=0):
        """
            Draw a connected path from start of route to end.
            Green node = start
            Red node   = point along path
            Blue node  = destination
        """
        for i in range(0, len(self.route_waypoints)-1, skip+1):
            w0 = self.route_waypoints[i][0]
            w1 = self.route_waypoints[i+1][0]
            self.world.debug.draw_line(
                w0.transform.location + carla.Location(z=0.25),
                w1.transform.location + carla.Location(z=0.25),
                thickness=0.1, color=carla.Color(255, 0, 0),
                life_time=life_time, persistent_lines=False)
            self.world.debug.draw_point(
                w0.transform.location + carla.Location(z=0.25), 0.1,
                carla.Color(0, 255, 0) if i == 0 else carla.Color(255, 0, 0),
                life_time, False)
        self.world.debug.draw_point(
            self.route_waypoints[-1][0].transform.location + carla.Location(z=0.25), 0.1,
            carla.Color(0, 0, 255),
            life_time, False)
        
    def _draw_points(self, life_time=60.0, waypoint_list=None):
        """
            Draw a connected path from start of route to end.
            Green node = start
            Red node   = point along path
            Blue node  = destination
        """
        for i in range(0, len(waypoint_list)-1, 2):
            self.debug_point.x = waypoint_list[i]
            self.debug_point.y = waypoint_list[i+1]
            self.world.debug.draw_point(
                self.debug_point + carla.Location(z=1.0), 0.1,
                carla.Color(0, 255, 0) if i == 0 else carla.Color(255, 0, 0),
                life_time, False)