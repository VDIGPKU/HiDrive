import copy
import logging
import numpy as np
import os
import time
from threading import Thread

from queue import Queue
from queue import Empty

import carla
import py_trees
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.timer import GameTime

# UE5 compatibility: carla.libcarla may not exist
try:
    _Image = carla.libcarla.Image
    _LidarMeasurement = carla.libcarla.LidarMeasurement
    _RadarMeasurement = carla.libcarla.RadarMeasurement
    _GnssMeasurement = carla.libcarla.GnssMeasurement
    _IMUMeasurement = carla.libcarla.IMUMeasurement
except AttributeError:
    _Image = carla.Image
    _LidarMeasurement = carla.LidarMeasurement
    _RadarMeasurement = carla.RadarMeasurement
    _GnssMeasurement = carla.GnssMeasurement
    _IMUMeasurement = carla.IMUMeasurement


def threaded(fn):
    def wrapper(*args, **kwargs):
        thread = Thread(target=fn, args=args, kwargs=kwargs)
        thread.setDaemon(True)
        thread.start()

        return thread
    return wrapper


class SensorConfigurationInvalid(Exception):
    """
    Exceptions thrown when the sensors used by the agent are not allowed for that specific submissions
    """

    def __init__(self, message):
        super(SensorConfigurationInvalid, self).__init__(message)


class SensorReceivedNoData(Exception):
    """
    Exceptions thrown when the sensors used by the agent take too long to receive data
    """

    def __init__(self, message):
        super(SensorReceivedNoData, self).__init__(message)


class GenericMeasurement(object):
    def __init__(self, data, frame):
        self.data = data
        self.frame = frame


class BaseReader(object):
    def __init__(self, vehicle, reading_frequency=1.0):
        self._vehicle = vehicle
        self._reading_frequency = reading_frequency
        self._callback = None
        self._run_ps = True
        self.run()

    def __call__(self):
        pass

    @threaded
    def run(self):
        first_time = True
        latest_time = GameTime.get_time()
        while self._run_ps:
            if self._callback is not None:
                current_time = GameTime.get_time()

                # Second part forces the sensors to send data at the first tick, regardless of frequency
                if current_time - latest_time > (1 / self._reading_frequency) \
                        or (first_time and GameTime.get_frame() != 0):
                    self._callback(GenericMeasurement(self.__call__(), GameTime.get_frame()))
                    latest_time = GameTime.get_time()
                    first_time = False

                else:
                    time.sleep(0.001)

    def listen(self, callback):
        # Tell that this function receives what the producer does.
        self._callback = callback

    def stop(self):
        self._run_ps = False

    def destroy(self):
        self._run_ps = False


class SpeedometerReader(BaseReader):
    """
    Sensor to measure the speed of the vehicle.
    """
    MAX_CONNECTION_ATTEMPTS = 10

    def _get_forward_speed(self, transform=None, velocity=None):
        """ Convert the vehicle transform directly to forward speed """
        if not velocity:
            velocity = self._vehicle.get_velocity()
        if not transform:
            transform = self._vehicle.get_transform()

        vel_np = np.array([velocity.x, velocity.y, velocity.z])
        pitch = np.deg2rad(transform.rotation.pitch)
        yaw = np.deg2rad(transform.rotation.yaw)
        orientation = np.array([np.cos(pitch) * np.cos(yaw), np.cos(pitch) * np.sin(yaw), np.sin(pitch)])
        speed = np.dot(vel_np, orientation)
        return speed

    def __call__(self):
        """ We convert the vehicle physics information into a convenient dictionary """

        # protect this access against timeout
        attempts = 0
        while attempts < self.MAX_CONNECTION_ATTEMPTS:
            try:
                velocity = self._vehicle.get_velocity()
                transform = self._vehicle.get_transform()
                break
            except Exception:
                attempts += 1
                time.sleep(0.2)
                continue

        return {'speed': self._get_forward_speed(transform=transform, velocity=velocity)}


class OpenDriveMapReader(BaseReader):
    def __call__(self):
        return {'opendrive': CarlaDataProvider.get_map().to_opendrive()}


class CallBack(object):
    def __init__(self, tag, sensor_type, sensor, data_provider):
        self._tag = tag
        self._data_provider = data_provider

        self._data_provider.register_sensor(tag, sensor_type, sensor)

    def __call__(self, data):
        # Get the class name for type checking (UE5 compatibility)
        data_type = type(data).__name__

        if isinstance(data, _Image) or data_type == 'Image':
            self._parse_image_cb(data, self._tag)
        elif isinstance(data, _LidarMeasurement) or data_type == 'LidarMeasurement':
            self._parse_lidar_cb(data, self._tag)
        elif isinstance(data, _RadarMeasurement) or data_type == 'RadarMeasurement':
            self._parse_radar_cb(data, self._tag)
        elif isinstance(data, _GnssMeasurement) or data_type == 'GnssMeasurement':
            self._parse_gnss_cb(data, self._tag)
        elif isinstance(data, _IMUMeasurement) or data_type == 'IMUMeasurement':
            self._parse_imu_cb(data, self._tag)
        elif isinstance(data, GenericMeasurement):
            self._parse_pseudosensor(data, self._tag)
        else:
            logging.error(f'No callback method for sensor type: {data_type}')

    # Parsing CARLA physical Sensors
    def _parse_image_cb(self, image, tag):
        import carla
        image.convert(carla.ColorConverter.Raw)

        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = copy.deepcopy(array)
        array = np.reshape(array, (image.height, image.width, 4))
        self._apply_camera_occlusion_if_needed(array, tag)
        self._data_provider.update_sensor(tag, array, image.frame)

    @staticmethod
    def _safe_blackboard_get(key, default=None):
        try:
            return py_trees.blackboard.Blackboard().get(key)
        except KeyError:
            return default
        except Exception:
            return default

    @staticmethod
    def _tokenize_camera_tag(raw_tag):
        """Normalize camera-tag aliases so scenario tags can match agent sensor ids."""
        token = str(raw_tag).strip().lower()
        aliases = {
            # Common single-front camera aliases used by scenarios/agents.
            "center": "front_center",
            "front": "front_center",
            "front_center": "front_center",
            "main": "front_center",
            "rgb_0": "front_center",
            "rgb0": "front_center",
            "cam0": "front_center",
            "camera0": "front_center",
        }
        return aliases.get(token, token)

    def _apply_camera_occlusion_if_needed(self, array, tag):
        active = bool(self._safe_blackboard_get("CameraOcclusion_active", False))
        if not active:
            return

        tags_raw = self._safe_blackboard_get("CameraOcclusion_tags", "Center")
        tags_str = str(tags_raw).strip()
        if tags_str and tags_str != "*":
            wanted = [self._tokenize_camera_tag(x) for x in tags_str.split(",") if x.strip()]
            tag_l = str(tag).lower()
            tag_n = self._tokenize_camera_tag(tag_l)
            if wanted and not any((w == tag_n) or (w in tag_l) for w in wanted):
                return

        h, w = array.shape[:2]
        x_ratio = float(self._safe_blackboard_get("CameraOcclusion_x_ratio", 0.55))
        y_ratio = float(self._safe_blackboard_get("CameraOcclusion_y_ratio", 0.20))
        w_ratio = float(self._safe_blackboard_get("CameraOcclusion_w_ratio", 0.25))
        h_ratio = float(self._safe_blackboard_get("CameraOcclusion_h_ratio", 0.35))

        x_ratio = max(0.0, min(1.0, x_ratio))
        y_ratio = max(0.0, min(1.0, y_ratio))
        w_ratio = max(0.0, min(1.0, w_ratio))
        h_ratio = max(0.0, min(1.0, h_ratio))

        x0 = max(0, min(w - 1, int(w * x_ratio)))
        y0 = max(0, min(h - 1, int(h * y_ratio)))
        occ_w = max(1, int(w * w_ratio))
        occ_h = max(1, int(h * h_ratio))
        x1 = max(x0 + 1, min(w, x0 + occ_w))
        y1 = max(y0 + 1, min(h, y0 + occ_h))

        array[y0:y1, x0:x1, :3] = 0
        if array.shape[2] > 3:
            array[y0:y1, x0:x1, 3] = 255

    def _parse_lidar_cb(self, lidar_data, tag):
        points = np.frombuffer(lidar_data.raw_data, dtype=np.dtype('f4'))
        points = copy.deepcopy(points)
        points = np.reshape(points, (int(points.shape[0] / 4), 4))
        self._data_provider.update_sensor(tag, points, lidar_data.frame)

    def _parse_radar_cb(self, radar_data, tag):
        # [depth, azimuth, altitute, velocity]
        points = np.frombuffer(radar_data.raw_data, dtype=np.dtype('f4'))
        points = copy.deepcopy(points)
        points = np.reshape(points, (int(points.shape[0] / 4), 4))
        points = np.flip(points, 1)
        self._data_provider.update_sensor(tag, points, radar_data.frame)

    def _parse_gnss_cb(self, gnss_data, tag):
        array = np.array([gnss_data.latitude,
                          gnss_data.longitude,
                          gnss_data.altitude], dtype=np.float64)
        self._data_provider.update_sensor(tag, array, gnss_data.frame)

    def _parse_imu_cb(self, imu_data, tag):
        array = np.array([imu_data.accelerometer.x,
                          imu_data.accelerometer.y,
                          imu_data.accelerometer.z,
                          imu_data.gyroscope.x,
                          imu_data.gyroscope.y,
                          imu_data.gyroscope.z,
                          imu_data.compass,
                         ], dtype=np.float64)
        self._data_provider.update_sensor(tag, array, imu_data.frame)

    def _parse_pseudosensor(self, package, tag):
        self._data_provider.update_sensor(tag, package.data, package.frame)


class SensorInterface(object):
    def __init__(self):
        self._sensors_objects = {}
        self._data_buffers = Queue()
        self._queue_timeout = 300

        # Only sensor that doesn't get the data on tick, needs special treatment
        self._opendrive_tag = None

    def register_sensor(self, tag, sensor_type, sensor):
        if tag in self._sensors_objects:
            raise SensorConfigurationInvalid("Duplicated sensor tag [{}]".format(tag))

        self._sensors_objects[tag] = sensor

        if sensor_type == 'sensor.opendrive_map': 
            self._opendrive_tag = tag

    def update_sensor(self, tag, data, frame):
        if tag not in self._sensors_objects:
            raise SensorConfigurationInvalid("The sensor with tag [{}] has not been created!".format(tag))

        self._data_buffers.put((tag, frame, data))

    def get_data(self, frame):
        """Read the queue to get the sensors data"""
        try:
            data_dict = {}
            while len(data_dict.keys()) < len(self._sensors_objects.keys()):
                # Don't wait for the opendrive sensor
                if self._opendrive_tag and self._opendrive_tag not in data_dict.keys() \
                        and len(self._sensors_objects.keys()) == len(data_dict.keys()) + 1:
                    break

                sensor_data = self._data_buffers.get(True, self._queue_timeout)
                if sensor_data[1] != frame:
                    continue
                data_dict[sensor_data[0]] = ((sensor_data[1], sensor_data[2]))

        except Empty:
            raise SensorReceivedNoData("A sensor took too long to send their data")

        return data_dict
