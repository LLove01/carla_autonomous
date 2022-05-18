import queue
import math
import glob
import os
import sys
import numpy as np

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
import carla


def get_speed(vehicle):
    velocity = vehicle.get_velocity()
    return 3.6*math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)


class PID_Controller():
    def __init__(self, vehicle, args_acceleration, args_direction, max_throttle=0.75, max_break=0.3, max_steering=0.8):
        self.max_break = max_break
        self.max_throttle = max_throttle
        self.max_steering = max_steering
        self.vehicle = vehicle

        self.prev_steering = self.vehicle.get_control().steer
        self.world = vehicle.get_world()
        self.accel_controller = PID_Acceleration_Control(
            self.vehicle, **args_acceleration)
        self.dir_controller = PID_Direction_Control(self.vehicle, **args_direction)

    def run_step(self, target_speed, waypoint):

        acceleration = self.accel_controller.run_step(target_speed)
        current_steering = self.dir_controller.run_step(waypoint)
        control = carla.VehicleControl()
        # acceleration
        if acceleration >= 0.0:
            control.throttle = min(abs(acceleration), self.max_break)
            control.brake = 0.0
        else:
            control.throttle = 0.0
            control.brake = min(abs(acceleration), self.max_break)
        # steering
        if current_steering > self.prev_steering + 0.1:
            current_steering = self.prev_steering + 0.1
        elif current_steering < self.prev_steering - 0.1:
            current_steering = self.prev_steering - 0.1

        if current_steering >= 0:
            steering = min(self.max_steering, current_steering)
        else:
            steering = max(-self.max_steering, current_steering)
        # setting control values
        control.steer = steering
        control.hand_brake = False
        control.manual_gear_shift = False
        # setting previous steering
        self.prev_steering = steering
        return control


class PID_Acceleration_Control():
    def __init__(self, vehicle, K_P=1.0, K_D=0.0, K_I=0.0, dt=0.03):
        self.vehicle = vehicle
        self.K_D = K_D
        self.K_P = K_P
        self.K_I = K_I
        self.dt = dt
        self.error_buffer = queue.deque(maxlen= 10)

    def run_step(self, target_speed):
        current_speed = get_speed(self.vehicle)
        return self.acceleration_controller(target_speed, current_speed)

    def acceleration_controller(self, target_speed, current_speed):
        diff = target_speed - current_speed
        self.error_buffer.append(diff)

        if len(self.error_buffer) >= 2:
            de = (self.error_buffer[-1] - self.error_buffer[-2])/self.dt
            ie = sum(self.error_buffer) * self.dt
        else:
            de = 0.0
            ie = 0.0
        # normalizing
        return np.clip(self.K_P*diff + self.K_D*de + self.K_I*ie, -1.0, 1.0)


class PID_Direction_Control():
    def __init__(self, vehicle, K_P=1.0, K_D=0.0, K_I=0.0, dt=0.03):
        self.vehicle = vehicle
        self.K_D = K_D
        self.K_P = K_P
        self.K_I = K_I
        self.dt = dt
        self.error_buffer = queue.deque(maxlen=10)

    def run_step(self, waypoint):
        return self.direction_controller(waypoint, self.vehicle.get_transform())

    def direction_controller(self, waypoint, vehicle_transform):
        v_begin = vehicle_transform.location
        v_end = v_begin + carla.Location(x=math.cos(math.radians(
            vehicle_transform.rotation.yaw)), y=math.sin(math.radians(vehicle_transform.rotation.yaw)))
        v_vec = np.array([v_end.x - v_begin.x, v_end.y - v_begin.y, 0.0])
        w_vec = np.array([waypoint.transform.location.x - v_begin.x,
                         waypoint.transform.location.y - v_begin.y, 0.0])
        dot = math.acos(np.clip(np.dot(w_vec, v_vec) /
                        np.linalg.norm(w_vec)*np.linalg.norm(v_vec), -1.0, 1.0))
        cross = np.cross(v_vec, w_vec)

        if cross[2] < 0:
            dot *= -1
        self.error_buffer.append(dot)

        if len(self.error_buffer) >= 2:
            de = (self.error_buffer[-1] - self.error_buffer[-2]) / self.dt
            ie = sum(self.error_buffer)*self.dt
        else:
            de = 0.0
            ie = 0.0
        # normalizing
        return np.clip((self.K_P*dot) + (self.K_I*ie) + (self.K_D*de), -1.0, 1.0)
