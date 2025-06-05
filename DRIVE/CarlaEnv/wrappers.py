import carla
import math
import numpy as np
import weakref
import pygame

def print_transform(transform):
    print("Location(x={:.2f}, y={:.2f}, z={:.2f}) Rotation(pitch={:.2f}, yaw={:.2f}, roll={:.2f})".format(
            transform.location.x,
            transform.location.y,
            transform.location.z,
            transform.rotation.pitch,
            transform.rotation.yaw,
            transform.rotation.roll
        )
    ) # 打印位姿信息

def get_actor_display_name(actor, truncate=250):
    name = " ".join(actor.type_id.replace("_", ".").title().split(".")[1:])
    return (name[:truncate-1] + u"\u2026") if len(name) > truncate else name

def angle_diff(v0, v1):
    """ Calculates the signed angle difference (-pi, pi] between 2D vector v0 and v1 """
    angle = np.arctan2(v1[1], v1[0]) - np.arctan2(v0[1], v0[0])
    if angle > np.pi: angle -= 2 * np.pi
    elif angle <= -np.pi: angle += 2 * np.pi
    return angle # 计算两个向量的角度差

def distance_to_line(A, B, p):
    num   = np.linalg.norm(np.cross(B - A, A - p))
    denom = np.linalg.norm(B - A)
    if np.isclose(denom, 0):
        return np.linalg.norm(p - A)
    return num / denom # 计算到直线的距离

def vector(v):
    """ Turn carla Location/Vector3D/Rotation to np.array """
    if isinstance(v, carla.Location) or isinstance(v, carla.Vector3D):
        return np.array([v.x, v.y, v.z])
    elif isinstance(v, carla.Rotation):
        return np.array([v.pitch, v.yaw, v.roll])



def angle_diff_(v0, v1):
    """
    Calculates the signed angle difference between 2D vectors v0 and v1.
    It returns the angle difference in radians between v0 and v1.
    The v0 is the reference for the sign of the angle
    """
    v0_xy = v0[:2]
    v1_xy = v1[:2]
    v0_xy_norm = np.linalg.norm(v0_xy)
    v1_xy_norm = np.linalg.norm(v1_xy)
    if v0_xy_norm == 0 or v1_xy_norm == 0:
        return 0

    v0_xy_u = v0_xy / v0_xy_norm
    v1_xy_u = v1_xy / v1_xy_norm
    dot_product = np.dot(v0_xy_u, v1_xy_u)
    angle = np.arccos(dot_product)

    # Calculate the sign of the angle using the cross product
    cross_product = np.cross(v0_xy_u, v1_xy_u)
    if cross_product < 0:
        angle = -angle
    if abs(angle) >= 2.3:
        return 0
    return round(angle, 2)

def distance_to_line_sgn(pointA, pointB, pointP): # 没有考虑三维，在一些地图中可能不适用，比如Town07
    if abs(pointA[0] - pointB[0]) < 0.01:
        return pointP[0] - pointA[0]
    A = pointA[1] - pointB[1] # y1 - y2
    B = pointB[0] - pointA[0] # x2 - x1
    C = pointA[0] * pointB[1] - pointA[1] * pointB[0] # x1 * y2 - y1 * x2
    distance = abs(A * pointP[0] + B * pointP[1] + C) / math.sqrt(A * A + B * B)
    if pointP[0] > (-(B * pointP[1] + C) / A):
        return distance
    else:
        return -distance

def get_displacement_vector(car_pos, waypoint_pos, theta):
    """
    Calculates the displacement vector from the car to a waypoint, taking into account the orientation of the car.

    Parameters:
        car_pos (numpy.ndarray): 1D numpy array of shape (3,) representing the x, y, z coordinates of the car.
        waypoint_pos (numpy.ndarray): 1D numpy array of shape (3,) representing the x, y, z coordinates of the waypoint.
        theta (float): Angle in radians representing the orientation of the car.

    Returns:
        numpy.ndarray: 1D numpy array of shape (3,) representing the displacement vector from the car to the waypoint,
        with the car as the origin and the y-axis pointing in the direction of the car's orientation.
    """
    # Calculate the relative position of the waypoint with respect to the car
    relative_pos = waypoint_pos - car_pos

    theta = theta
    # Construct the rotation transformation matrix
    R = np.array([[np.cos(theta), np.sin(theta), 0],
                  [-np.sin(theta), np.cos(theta), 0],
                  [0, 0, 1]])
    T = np.array([[0, 1, 0],
                  [1, 0, 0],
                  [0, 0, 1]])
    # Apply the rotation matrix to the relative position vector
    waypoint_car = R @ relative_pos
    waypoint_car = T @ waypoint_car
    # Set values very close to zero to exactly zero
    waypoint_car[np.abs(waypoint_car) < 10e-10] = 0

    return waypoint_car


camera_transforms = {
    "spectator": carla.Transform(carla.Location(x=-5.5, z=2.8), carla.Rotation(pitch=-15)),
    "dashboard": carla.Transform(carla.Location(x=1.6, z=1.7))
}

#===============================================================================
# CarlaActorBase
#===============================================================================

class CarlaActorBase(object):
    def __init__(self, world, actor):
        self.world = world
        self.actor = actor
        self.world.actor_list.append(self)
        self.destroyed = False

    def destroy(self):
        if self.destroyed:
            raise Exception("Actor already destroyed.")
        else:
            print("Destroying ", self, "...")
            self.actor.destroy()
            self.world.actor_list.remove(self)
            self.destroyed = True

    def get_carla_actor(self):
        return self.actor

    def tick(self):
        pass

    def __getattr__(self, name):
        """Relay missing methods to underlying carla actor"""
        return getattr(self.actor, name)

#===============================================================================
# CollisionSensor
#===============================================================================

class CollisionSensor(CarlaActorBase):
    def __init__(self, world, vehicle, on_collision_fn):
        self.on_collision_fn = on_collision_fn

        # Collision history
        self.collision_data = list()

        # Setup sensor blueprint
        bp = world.get_blueprint_library().find("sensor.other.collision") # 碰撞传感器

        # Create and setup sensor
        weak_self = weakref.ref(self)
        actor = world.spawn_actor(bp, carla.Transform(carla.Location(x=1.3, z=0.5)), attach_to=vehicle.get_carla_actor()) # 在 Carla 世界中创建一个新的 Actor，并设置其位姿，并附加到车上
        actor.listen(lambda event: CollisionSensor.on_collision(weak_self, event)) # 碰撞传感器回调函数

        super().__init__(world, actor)

    @staticmethod
    def on_collision(weak_self, event): # 执行自定义逻辑，例如记录碰撞信息、终止模拟或触发其他行为
        self = weak_self()
        if not self:
            return
        # impulse = event.normal_impulse
        # intensity = math.sqrt(impulse.x ** 2 + impulse.y ** 2 + impulse.z ** 2)
        # self.collision_data.append(intensity)

        if callable(self.on_collision_fn):
            self.on_collision_fn(event)


#===============================================================================
# LaneInvasionSensor
#===============================================================================

class LaneInvasionSensor(CarlaActorBase):
    def __init__(self, world, vehicle, on_invasion_fn):
        self.on_invasion_fn = on_invasion_fn

        # Setup sensor blueprint
        bp = world.get_blueprint_library().find("sensor.other.lane_invasion")

        # Create sensor
        weak_self = weakref.ref(self)
        actor = world.spawn_actor(bp, carla.Transform(), attach_to=vehicle.get_carla_actor())
        actor.listen(lambda event: LaneInvasionSensor.on_invasion(weak_self, event))

        super().__init__(world, actor)

    @staticmethod
    def on_invasion(weak_self, event):
        self = weak_self()
        if not self:
            return

        # Call on_invasion_fn
        if callable(self.on_invasion_fn):
            self.on_invasion_fn(event)

#===============================================================================
# Camera
#===============================================================================

class Camera(CarlaActorBase):
    def __init__(self, world, width, height, transform=carla.Transform(),
                 sensor_tick=0.0, attach_to=None, on_recv_image=None,
                 camera_type="sensor.camera.rgb", color_converter=carla.ColorConverter.Raw, custom_palette=False):
        self.on_recv_image = on_recv_image
        self.color_converter = color_converter

        self.custom_palette = custom_palette # LJW:

        # Setup camera blueprint
        camera_bp = world.get_blueprint_library().find(camera_type) # 相机类型RGB相机
        camera_bp.set_attribute("image_size_x", str(width))
        camera_bp.set_attribute("image_size_y", str(height))
        camera_bp.set_attribute("sensor_tick", str(sensor_tick)) # 相机传感器触发频率，0.0表示尽可能高频，甚至与仿真频率一致

        # Create and setup camera actor
        weak_self = weakref.ref(self)
        actor = world.spawn_actor(camera_bp, transform, attach_to=attach_to.get_carla_actor())
        actor.listen(lambda image: Camera.process_camera_input(weak_self, image))
        print("Spawned actor \"{}\"".format(actor.type_id))

        super().__init__(world, actor)
    
    @staticmethod
    def process_camera_input(weak_self, image):
        self = weak_self()
        if not self:
            return
        if callable(self.on_recv_image):
            image.convert(self.color_converter)
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]

            # LJW:
            if self.custom_palette:
                classes = {
                    0: [0, 0, 0],  # None
                    1: [0, 0, 0],  # Buildings
                    2: [0, 0, 0],  # Fences
                    3: [0, 0, 0],  # Other
                    4: [0, 0, 0],  # Pedestrians
                    5: [0, 0, 0],  # Poles
                    6: [157, 234, 50],  # RoadLines
                    7: [50, 64, 128],  # Roads
                    8: [255, 255, 255],  # Sidewalks
                    9: [0, 0, 0],  # Vegetation
                    10: [0, 0, 0],  # Vehicles
                    11: [0, 0, 0],  # Walls
                    12: [0, 0, 0],  # TrafficSigns
                    13: [0, 0, 0],  # Sky
                }
                # classes = {
                #     0: [0, 0, 0],  # None
                #     1: [70, 70, 70],  # Buildings
                #     2: [100, 40, 40],  # Fences
                #     3: [55, 90, 80],  # Other
                #     4: [220, 20, 60],  # Pedestrians
                #     5: [153, 153, 153],  # Poles
                #     6: [157, 234, 50],  # RoadLines
                #     7: [128, 64, 128],  # Roads
                #     8: [244, 35, 232],  # Sidewalks
                #     9: [107, 142, 35],  # Vegetation (森林、树木等)
                #     10: [0, 0, 142],  # Vehicles
                #     11: [102, 102, 156],  # Walls
                #     12: [220, 220, 0],  # TrafficSigns
                #     13: [70, 130, 180],  # Sky
                #     14: [81, 0, 81],  # Ground
                #     15: [150, 100, 100],  # Bridge
                #     16: [230, 150, 140],  # RailTrack
                #     17: [180, 165, 180],  # GuardRail
                #     18: [250, 170, 30],  # TrafficLight
                #     19: [110, 190, 160],  # Static
                #     20: [170, 120, 50],  # Dynamic
                #     21: [45, 60, 150],  # Water
                #     22: [145, 170, 100],  # Terrain (草地、沙地等)
                # }
                segimg = np.round((array[:, :, 0])).astype(np.uint8)
                array = array.copy()
                for j in range(array.shape[0]):
                    for i in range(array.shape[1]):
                        r_id = segimg[j, i]
                        if r_id <= 12:
                            array[j, i] = classes[segimg[j, i]]
                        else:
                            array[j, i] = classes[0]

            self.on_recv_image(array)

    def destroy(self):
        super().destroy()

#===============================================================================
# Vehicle
#===============================================================================

class Vehicle(CarlaActorBase):
    def __init__(self, world, transform=carla.Transform(),
                 on_collision_fn=None, on_invasion_fn=None,
                 vehicle_type="vehicle.lincoln.mkz2017"):
        # Setup vehicle blueprint
        vehicle_bp = world.get_blueprint_library().find(vehicle_type) # 车辆类型
        color = vehicle_bp.get_attribute("color").recommended_values[0] # 车辆颜色
        vehicle_bp.set_attribute("color", color) # 设置车辆属性

        # Create vehicle actor
        actor = world.spawn_actor(vehicle_bp, transform) # 创建车辆，位置是transform
        print("Spawned actor \"{}\"".format(actor.type_id))
            
        super().__init__(world, actor)

        # Maintain vehicle control
        self.control = carla.VehicleControl() # 车辆控制器

        if callable(on_collision_fn):
            self.collision_sensor = CollisionSensor(world, self, on_collision_fn=on_collision_fn) # 碰撞传感器
        if callable(on_invasion_fn):
            self.lane_sensor = LaneInvasionSensor(world, self, on_invasion_fn=on_invasion_fn) # 道路侵犯传感器

    def tick(self):
        self.actor.apply_control(self.control)

    def get_speed(self):
        velocity = self.get_velocity()
        return np.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)

    def get_closest_waypoint(self):
        return self.world.map.get_waypoint(self.get_transform().location, project_to_road=True)

    def get_angle(self, waypoint):
        fwd = vector(self.get_velocity())
        wp_fwd = vector(waypoint.transform.rotation.get_forward_vector())
        return angle_diff_(wp_fwd, fwd)

#===============================================================================
# World
#===============================================================================

class World():
    def __init__(self, client):
        self.world = client.get_world()
        self.map = self.get_map()
        self.actor_list = []

    def tick(self):
        for actor in list(self.actor_list):
            actor.tick()
        self.world.tick()

    def destroy(self):
        print("Destroying all spawned actors")
        for actor in list(self.actor_list):
            actor.destroy()

    def get_carla_world(self):
        return self.world

    def __getattr__(self, name):
        """Relay missing methods to underlying carla object"""
        return getattr(self.world, name)
