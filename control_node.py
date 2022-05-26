import glob
import os
import sys
from matplotlib import image
from matplotlib.colors import ColorConverter
import numpy as np
import cv2
import queue
import random

from pid_controller import *
from tf_detection import *

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
import carla


def reset(self):
    print("Collision! Respawning ...")
    self.vehicle_bp = self.bp_lib.filter('cybertruck')[0]
    # spawn_point = carla.Transform(carla.Location(
    #     x=-75.4, y=-1.0, z=15), carla.Rotation(pitch=0, yaw=180, roll=0))
    # spawn_point = random.choice(world.get_map().get_spawn_points())
    self.spawn_point = self.world.get_map().get_spawn_points()[3]
    self.vehicle = self.world.spawn_actor(
        self.vehicle_bp, self.spawn_point)
    self.actor_list.append(self.vehicle)
    print("Car spawned")
    # rgb camera
    camera_bp = self.bp_lib.find('sensor.camera.rgb')
    camera_bp.set_attribute('image_size_x', '800')
    camera_bp.set_attribute('image_size_y', '600')
    camera_bp.set_attribute('fov', '90')
    camera_location = carla.Transform(carla.Location(x=1.5, z=2.4))
    self.camera = self.world.spawn_actor(
        camera_bp, camera_location, attach_to=self.vehicle)
    self.camera.listen(lambda image: process_image(image))
    self.actor_list.append(self.camera)
    # collision sensor
    collision_bp = self.world.get_blueprint_library().find('sensor.other.collision')
    self.collision_sensor = self.world.spawn_actor(
        collision_bp, carla.Transform(), attach_to=self.vehicle)
    self.collision_sensor.listen(lambda event: on_collision(event, self))
    self.actor_list.append(self.collision_sensor)


def on_collision(event, self):
    self.client.apply_batch(
        [carla.command.DestroyActor(x) for x in self.actor_list])
    self.reset()


def process_image(image):
    # rgb camera returns 4 chanels
    image = np.array(image.raw_data)
    img = image.reshape((600, 800, 4))
    # cutting out alpha chanel
    img = img[:, :, :3]
    # black background
    stencil = np.zeros_like(img[:, :, 0])
    # Defining area of interest
    polygon = np.array([[50, 570], [50, 360], [780, 360], [780, 570]])
    cv2.fillConvexPoly(stencil, polygon, (255))
    mask_img = cv2.bitwise_and(img[:, :, 0], img[:, :, 0], mask=stencil)
    # cv2.imshow('Mask image', mask_img)
    ret, thresh = cv2.threshold(mask_img, 210, 255, cv2.THRESH_BINARY)

    #  Opening morphological operation
    ker1 = np.ones((3, 3), np.uint8)
    filtered = cv2.morphologyEx(thresh, cv2.MORPH_DILATE, ker1)
    # cv2.imshow('Dilated', filtered)

    blur = cv2.GaussianBlur(thresh, (13, 13), 0)
    thresh = cv2.threshold(blur, 100, 255, cv2.THRESH_BINARY)[1]

    cv2.imshow('Tresh', thresh)
    lines = cv2.HoughLinesP(thresh, 1, np.pi/180, 30, maxLineGap=200)

    # converting rgb to gray
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(img_gray, (x1, y1), (x2, y2), (255, 0, 0), 3)
    # tf object detection
    img = show_inference(detection_model, img)
    # cv2.imshow("rgb cam", img)
    # cv2.imshow("gray image", img_gray)
    cv2.waitKey(50)
    return img


class Cybertruck:
    def __init__(self):
        self.actor_list = []

        self.client = carla.Client('127.0.0.1', 3000)
        self.client.set_timeout(5.0)
        print("connected")
        self.world = self.client.get_world()
        self.map = self.world.get_map()
        self.bp_lib = self.world.get_blueprint_library()
        # creating and spawning a vehicle
        self.vehicle_bp = self.bp_lib.filter('cybertruck')[0]
        # spawn_point = carla.Transform(carla.Location(
        #     x=-75.4, y=-1.0, z=15), carla.Rotation(pitch=0, yaw=180, roll=0))
        # spawn_point = random.choice(world.get_map().get_spawn_points())
        self.spawn_point = self.world.get_map().get_spawn_points()[3]
        self.vehicle = self.world.spawn_actor(
            self.vehicle_bp, self.spawn_point)
        self.actor_list.append(self.vehicle)
        print("Car spawned")

        self.vehicle_controller = PID_Controller(self.vehicle, args_acceleration={
            'K_P': 1, 'K_D': 0.0, 'K_I': 0.0}, args_direction={'K_P': 1, 'K_D': 0.0, 'K_I': 0.0})

        # rgb camera
        camera_bp = self.bp_lib.find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', '800')
        camera_bp.set_attribute('image_size_y', '600')
        camera_bp.set_attribute('fov', '90')
        camera_location = carla.Transform(carla.Location(x=1.5, z=2.4))
        self.camera = self.world.spawn_actor(
            camera_bp, camera_location, attach_to=self.vehicle)
        self.camera.listen(lambda image: process_image(image))
        self.actor_list.append(self.camera)

        # collision sensor
        collision_bp = self.world.get_blueprint_library().find('sensor.other.collision')
        self.collision_sensor = self.world.spawn_actor(
            collision_bp, carla.Transform(), attach_to=self.vehicle)
        self.collision_sensor.listen(lambda event: on_collision(event, self))
        self.actor_list.append(self.collision_sensor)

    def run(self):
        try:
            while True:
                waypoints = self.world.get_map().get_waypoint(self.vehicle.get_location())
                waypoint = np.random.choice(waypoints.next(0.3))
                control_signal = self.vehicle_controller.run_step(30, waypoint)
                self.vehicle.apply_control(control_signal)
        finally:
            print('destroying actors')
            self.client.apply_batch(
                [carla.command.DestroyActor(x) for x in self.actor_list])
            print('done.')


if __name__ == '__main__':
    my_tesla = Cybertruck()
    my_tesla.run()
