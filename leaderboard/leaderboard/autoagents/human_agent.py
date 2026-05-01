#!/usr/bin/env python

# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module provides a human agent to control the ego vehicle via keyboard
"""

import numpy as np
import json
import math
import time

try:
    import pygame
    from pygame.locals import K_DOWN
    from pygame.locals import K_LEFT
    from pygame.locals import K_RIGHT
    from pygame.locals import K_SPACE
    from pygame.locals import K_UP
    from pygame.locals import K_a
    from pygame.locals import K_d
    from pygame.locals import K_s
    from pygame.locals import K_w
    from pygame.locals import K_q
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

import carla

from leaderboard.autoagents.autonomous_agent import AutonomousAgent, Track


def get_entry_point():
    return 'HumanAgent'

class HumanInterface(object):

    """
    Class to control a vehicle manually for debugging purposes
    """

    def __init__(self, width, height, side_scale, left_mirror=False, right_mirror=False):
        self._width = width
        self._height = height
        self._scale = side_scale
        self._surface = None
        self._overlay_lines = []

        self._left_mirror = left_mirror
        self._right_mirror = right_mirror

        pygame.init()
        pygame.font.init()
        self._font = pygame.font.SysFont("monospace", 20)
        self._clock = pygame.time.Clock()
        self._display = pygame.display.set_mode((self._width, self._height), pygame.HWSURFACE | pygame.DOUBLEBUF)
        pygame.display.set_caption("Human Agent")

    def set_overlay_lines(self, lines):
        self._overlay_lines = lines or []

    def run_interface(self, input_data):
        """
        Run the GUI
        """

        # Process sensor data
        image_center = input_data['Center'][1][:, :, -2::-1]
        self._surface = pygame.surfarray.make_surface(image_center.swapaxes(0, 1))

        # Add the left mirror
        if self._left_mirror:
            image_left = input_data['Left'][1][:, :, -2::-1]
            left_surface = pygame.surfarray.make_surface(image_left.swapaxes(0, 1))
            self._surface.blit(left_surface, (0, (1 - self._scale) * self._height))

        # Add the right mirror
        if self._right_mirror:
            image_right = input_data['Right'][1][:, :, -2::-1]
            right_surface = pygame.surfarray.make_surface(image_right.swapaxes(0, 1))
            self._surface.blit(right_surface, ((1 - self._scale) * self._width, (1 - self._scale) * self._height))

        # Display image
        if self._surface is not None:
            self._display.blit(self._surface, (0, 0))

        if self._overlay_lines:
            overlay_h = 26 * len(self._overlay_lines) + 8
            overlay = pygame.Surface((self._width, overlay_h), pygame.SRCALPHA)
            overlay.fill((0, 0, 0, 120))
            self._display.blit(overlay, (0, 0))

            y = 6
            for line in self._overlay_lines:
                text = self._font.render(line, True, (240, 240, 240))
                self._display.blit(text, (10, y))
                y += 24
        pygame.display.flip()

    def set_black_screen(self):
        """Set the surface to black"""
        black_array = np.zeros([self._width, self._height])
        self._surface = pygame.surfarray.make_surface(black_array)
        if self._surface is not None:
            self._display.blit(self._surface, (0, 0))
        pygame.display.flip()

    def _quit(self):
        pygame.quit()


class HumanAgent(AutonomousAgent):

    """
    Human agent to control the ego vehicle via keyboard
    """

    current_control = None
    agent_engaged = False

    def setup(self, path_to_conf_file):
        """
        Setup the agent parameters
        """
        self.track = Track.SENSORS

        self.agent_engaged = False
        self.camera_width = 1280
        self.camera_height = 720
        self._side_scale = 0.3
        self._left_mirror = False
        self._right_mirror = False

        import os
        self._bird_view = os.environ.get('BIRD_VIEW_MODE') == 'bird'
        if self._bird_view:
            print("=" * 50)
            print("BIRD VIEW MODE ENABLED")
            print("=" * 50)

        self._hic = HumanInterface(
            self.camera_width,
            self.camera_height,
            self._side_scale,
            self._left_mirror,
            self._right_mirror
        )
        self._controller = KeyboardControl(path_to_conf_file)
        self._prev_timestamp = 0
        self._last_coord_print = 0.0

        self._clock = pygame.time.Clock()

    def _get_ego_vehicle(self):
        try:
            if not hasattr(self, '_world'):
                from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
                self._world = CarlaDataProvider.get_world()

            if not self._world:
                return None

            for actor in self._world.get_actors().filter('vehicle.*'):
                if actor.attributes.get('role_name') == 'hero':
                    return actor
        except RuntimeError:
            return None
        return None

    def sensors(self):
        """
        Define the sensor suite required by the agent

        :return: a list containing the required sensors in the following format:

        [
            {'type': 'sensor.camera.rgb', 'x': 0.7, 'y': -0.4, 'z': 1.60, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
                      'width': 300, 'height': 200, 'fov': 100, 'id': 'Left'},

            {'type': 'sensor.camera.rgb', 'x': 0.7, 'y': 0.4, 'z': 1.60, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
                      'width': 300, 'height': 200, 'fov': 100, 'id': 'Right'},

            {'type': 'sensor.lidar.ray_cast', 'x': 0.7, 'y': 0.0, 'z': 1.60, 'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0,
             'id': 'LIDAR'}
        ]
        """

        if self._bird_view:
            sensors = [
                {'type': 'sensor.camera.rgb', 'x': 0.0, 'y': 0.0, 'z': 50.0,
                 'roll': 0.0, 'pitch': -90.0, 'yaw': 0.0,
                 'width': self.camera_width, 'height': self.camera_height,
                 'fov': 90, 'id': 'Center'},
            ]
        else:
            sensors = [
                {'type': 'sensor.camera.rgb', 'x': -2.0, 'y': 0.0, 'z': 2.0,
                 'roll': 0.0, 'pitch': -10.0, 'yaw': 0.0,
                 'width': self.camera_width, 'height': self.camera_height,
                 'fov': 90, 'id': 'Center'},
            ]

        if self._left_mirror:
            sensors.append(
                {'type': 'sensor.camera.rgb', 'x': 0.7, 'y': -1.0, 'z': 1, 'roll': 0.0, 'pitch': 0.0, 'yaw': 210.0,
                 'width': self.camera_width * self._side_scale, 'height': self.camera_height * self._side_scale,
                 'fov': 100, 'id': 'Left'})

        if self._right_mirror:
            sensors.append(
                {'type': 'sensor.camera.rgb', 'x': 0.7, 'y': 1.0, 'z': 1, 'roll': 0.0, 'pitch': 0.0, 'yaw': 150.0,
                 'width': self.camera_width * self._side_scale, 'height': self.camera_height * self._side_scale,
                 'fov': 100, 'id': 'Right'})

        return sensors

    def run_step(self, input_data, timestamp):
        """
        Execute one step of navigation.
        """
        self._clock.tick_busy_loop(20)
        self.agent_engaged = True

        ego_vehicle = self._get_ego_vehicle()
        if ego_vehicle is not None:
            transform = ego_vehicle.get_transform()
            location = transform.location
            rotation = transform.rotation
            velocity = ego_vehicle.get_velocity()
            speed_kmh = 3.6 * math.sqrt(
                velocity.x * velocity.x + velocity.y * velocity.y + velocity.z * velocity.z
            )
            self._hic.set_overlay_lines([
                f"XYZ: x={location.x:.2f}, y={location.y:.2f}, z={location.z:.2f}",
                f"Yaw={rotation.yaw:.1f} deg | Speed={speed_kmh:.1f} km/h",
            ])

            now = time.time()
            if now - self._last_coord_print >= 0.25:
                self._last_coord_print = now
                print(
                    f"XYZ: x={location.x:.2f}, y={location.y:.2f}, z={location.z:.2f} | "
                    f"yaw={rotation.yaw:.1f} | speed={speed_kmh:.1f} km/h",
                    flush=True,
                )
        else:
            self._hic.set_overlay_lines(["XYZ: waiting for hero vehicle..."])

        self._hic.run_interface(input_data)

        # Set bird view camera if enabled
        if self._bird_view:
            try:
                if self._world:
                    spectator = self._world.get_spectator()

                    if ego_vehicle:
                        transform = ego_vehicle.get_transform()
                        spectator.set_transform(carla.Transform(
                            transform.location + carla.Location(z=50),
                            carla.Rotation(pitch=-90)))
            except:
                pass

        control = self._controller.parse_events(timestamp - self._prev_timestamp)
        self._prev_timestamp = timestamp

        return control

    def destroy(self):
        """
        Cleanup
        """
        self._hic.set_black_screen()
        self._hic._quit = True


class KeyboardControl(object):

    """
    Keyboard control for the human agent
    """

    def __init__(self, path_to_conf_file):
        """
        Init
        """
        self._control = carla.VehicleControl()
        self._steer_cache = 0.0
        self._clock = pygame.time.Clock()

        # Get the mode
        if path_to_conf_file:
            # Check if it's a valid file path (not just a save_name suffix)
            import os
            if os.path.isfile(path_to_conf_file):
                with (open(path_to_conf_file, "r")) as f:
                    lines = f.read().split("\n")
                    self._mode = lines[0].split(" ")[1]
                    self._endpoint = lines[1].split(" ")[1]

                # Get the needed vars
                if self._mode == "log":
                    self._log_data = {'records': []}

                elif self._mode == "playback":
                    self._index = 0
                    self._control_list = []

                    with open(self._endpoint) as fd:
                        try:
                            self._records = json.load(fd)
                            self._json_to_control()
                        except json.JSONDecodeError:
                            pass
            else:
                # Not a valid file, use normal mode
                self._mode = "normal"
                self._endpoint = None
        else:
            self._mode = "normal"
            self._endpoint = None

    def _json_to_control(self):

        # transform strs into VehicleControl commands
        for entry in self._records['records']:
            control = carla.VehicleControl(throttle=entry['control']['throttle'],
                                           steer=entry['control']['steer'],
                                           brake=entry['control']['brake'],
                                           hand_brake=entry['control']['hand_brake'],
                                           reverse=entry['control']['reverse'],
                                           manual_gear_shift=entry['control']['manual_gear_shift'],
                                           gear=entry['control']['gear'])
            self._control_list.append(control)

    def parse_events(self, timestamp):
        """
        Parse the keyboard events and set the vehicle controls accordingly
        """
        # Move the vehicle
        if self._mode == "playback":
            self._parse_json_control()
        else:
            self._parse_vehicle_keys(pygame.key.get_pressed(), timestamp*1000)

        # Record the control
        if self._mode == "log":
            self._record_control()

        return self._control

    def _parse_vehicle_keys(self, keys, milliseconds):
        """
        Calculate new vehicle controls based on input keys
        """

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return 
            elif event.type == pygame.KEYUP:
                if event.key == K_q:
                    self._control.gear = 1 if self._control.reverse else -1
                    self._control.reverse = self._control.gear < 0

        if keys[K_UP] or keys[K_w]:
            self._control.throttle = 0.8
        else:
            self._control.throttle = 0.0

        steer_increment = 3e-4 * milliseconds
        if keys[K_LEFT] or keys[K_a]:
            self._steer_cache -= steer_increment
        elif keys[K_RIGHT] or keys[K_d]:
            self._steer_cache += steer_increment
        else:
            self._steer_cache = 0.0

        self._control.steer = round(self._steer_cache, 1)
        self._control.brake = 1.0 if keys[K_DOWN] or keys[K_s] else 0.0
        self._control.hand_brake = keys[K_SPACE]

    def _parse_json_control(self):

        if self._index < len(self._control_list):
            self._control = self._control_list[self._index]
            self._index += 1
        else:
            print("JSON file has no more entries")

    def _record_control(self):
        new_record = {
            'control': {
                'throttle': self._control.throttle,
                'steer': self._control.steer,
                'brake': self._control.brake,
                'hand_brake': self._control.hand_brake,
                'reverse': self._control.reverse,
                'manual_gear_shift': self._control.manual_gear_shift,
                'gear': self._control.gear
            }
        }

        self._log_data['records'].append(new_record)

    def __del__(self):
        # Get ready to log user commands
        if self._mode == "log" and self._log_data:
            with open(self._endpoint, 'w') as fd:
                json.dump(self._log_data, fd, indent=4, sort_keys=True)
