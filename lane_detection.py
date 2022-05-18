import glob
import os
import sys
from tkinter import Frame
from matplotlib import image
from matplotlib.colors import ColorConverter
import numpy as np
import cv2
import queue
import random

from pid_controller import *

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
import carla


def main():
    actor_list = []
    try:
        client = carla.Client('127.0.0.1', 2000)
        client.set_timeout(5.0)
        print("connected")
        world = client.get_world()
        map = world.get_map()
        bp_lib = world.get_blueprint_library()
        # creating and spawning a vehicle
        vehicle_bp = bp_lib.filter('cybertruck')[0]
        # spawn_point = carla.Transform(carla.Location(
        #     x=-75.4, y=-1.0, z=15), carla.Rotation(pitch=0, yaw=180, roll=0))
        # spawn_point = random.choice(world.get_map().get_spawn_points())
        spawn_point = world.get_map().get_spawn_points()[3]
        vehicle = world.spawn_actor(vehicle_bp, spawn_point)
        actor_list.append(vehicle)
        print("Car spawned")

        vehicle_controller = PID_Controller(vehicle, args_acceleration={
                                            'K_P': 1, 'K_D': 0.0, 'K_I': 0.0}, args_direction={'K_P': 1, 'K_D': 0.0, 'K_I': 0.0})

        while True:
            waypoints = world.get_map().get_waypoint(vehicle.get_location())
            waypoint = np.random.choice(waypoints.next(0.3))
            control_signal = vehicle_controller.run_step(50, waypoint)
            vehicle.apply_control(control_signal)

            # # semantic camera
            # camera_bp = bp_lib.find('sensor.camera.semantic_segmentation')
            # camera_bp.set_attribute('image_size_x', '800')
            # camera_bp.set_attribute('image_size_y', '600')
            # camera_bp.set_attribute('fov', '90')
            # camera_location = carla.Transform(carla.Location(x=1.5, y=2.4))
            # camera = world.spawn_actor(camera_bp, camera_location)
            # camera.listen(lambda image: image.save_to_disk(
            #     f'output/{image.frame}', carla.ColorConverter.CityScapesPalette))
            # # depth camera
            # depth_cam_bp = bp_lib.find('sensor.camera.depth')
            # depth_cam_location = carla.Transform(carla.Location(x=1.5, y=2.4))
            # camera = world.spawn_actor(depth_cam_bp, depth_cam_location)
            # camera.listen(lambda image: image.save_to_disk(
            #     f'output/{image.frame}', carla.ColorConverter.LogarithmicDepth))

    finally:
        print('destroying actors')
        for actor in actor_list:
            actor.destroy()
        print('done.')

if __name__ == '__main__':
    main()