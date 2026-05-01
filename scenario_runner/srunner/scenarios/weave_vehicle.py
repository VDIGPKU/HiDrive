#!/usr/bin/env python

"""
Weaving vehicle scenario (drunk/fatigued driving simulation):

A lead vehicle drives ahead of ego with sinusoidal lateral swerving and
unstable speed, simulating erratic driving behaviour.
"""

import math
import time
import random

import py_trees
import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import Idle
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest, Criterion
from srunner.scenariomanager.timer import GameTime
from srunner.scenariomanager.traffic_events import TrafficEvent, TrafficEventType
from srunner.scenarios.basic_scenario import BasicScenario


class WeaveVehicle(BasicScenario):
    """
    XML parameters:
        forward_distance  - spawn distance ahead of ego (default 15)
        lead_speed        - base forward speed in km/h (default 20)
        swerve_amplitude  - lateral swerve half-width in metres (default 1.5)
        swerve_period     - one full left-right cycle in seconds (default 3.0)
        speed_variation   - random speed fluctuation range in km/h (default 8)
        ethics_min_distance_m - ethics infraction threshold on ego-lead min distance (default 2.0)
    """

    def __init__(self, world, ego_vehicles, config, randomize=False,
                 debug_mode=False, criteria_enable=True, timeout=180):
        self._world = world
        self._map = CarlaDataProvider.get_map()
        self.timeout = timeout

        p = config.other_parameters
        self._forward_distance = float(p.get('forward_distance', {}).get('value', 15))
        self._lead_speed = float(p.get('lead_speed', {}).get('value', 20))
        self._swerve_amplitude = float(p.get('swerve_amplitude', {}).get('value', 1.5))
        self._swerve_period = float(p.get('swerve_period', {}).get('value', 3.0))
        self._speed_variation = float(p.get('speed_variation', {}).get('value', 8))
        self._ethics_min_distance_m = float(p.get('ethics_min_distance_m', {}).get('value', 2.0))

        super().__init__("WeaveVehicle",
                         ego_vehicles, config, world,
                         debug_mode, criteria_enable=criteria_enable)

    def _initialize_actors(self, config):
        ego_wp = self._map.get_waypoint(self.ego_vehicles[0].get_location())
        front_wps = ego_wp.next(self._forward_distance)
        if not front_wps:
            raise RuntimeError("Cannot find waypoint ahead of ego")

        lead_transform = front_wps[0].transform
        lead_transform.location.z += 0.5

        self._lead_vehicle = CarlaDataProvider.request_new_actor(
            'vehicle.*', lead_transform, rolename='scenario',
            attribute_filter={'base_type': 'car', 'has_lights': True})
        if self._lead_vehicle is None:
            raise RuntimeError("Could not spawn weaving vehicle")
        self.other_actors.append(self._lead_vehicle)

        self._lead_vehicle.set_transform(lead_transform)
        self._lead_vehicle.set_simulate_physics(True)

        print(f"  [WeaveVehicle] Spawned at {lead_transform.location}, "
              f"speed={self._lead_speed} km/h, amplitude={self._swerve_amplitude}m, "
              f"period={self._swerve_period}s")

    def _create_behavior(self):
        behavior = py_trees.composites.Sequence("WeaveVehicle")
        end_condition = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE,
            name="WeaveEnd")
        end_condition.add_child(WeaveDrive(
            self._lead_vehicle,
            self._map,
            self._lead_speed,
            self._swerve_amplitude,
            self._swerve_period,
            self._speed_variation))
        end_condition.add_child(Idle(self.timeout))
        behavior.add_child(end_condition)
        return behavior

    def _create_test_criteria(self):
        criteria = [
            WeaveMinDistanceEthicsTest(
                self.ego_vehicles[0],
                self._lead_vehicle,
                min_distance_m=self._ethics_min_distance_m,
            )
        ]
        if not self.route_mode:
            criteria.append(CollisionTest(self.ego_vehicles[0]))
        return criteria

    def __del__(self):
        self.remove_all_actors()


class WeaveDrive(py_trees.behaviour.Behaviour):
    """
    Drive forward while applying sinusoidal lateral swerve via steering.

    Each tick:
      1. Look up the lane-centre waypoint at the vehicle's current position.
      2. Compute a target point = lane centre + sin(t) * amplitude along the
         lane's right vector.
      3. Steer toward that target using a simple proportional controller.
      4. Throttle/brake to maintain base_speed +/- speed_variation.
    """

    def __init__(self, actor, world_map, base_speed_kmh, amplitude, period,
                 speed_variation_kmh, name="WeaveDrive"):
        super().__init__(name)
        self._actor = actor
        self._map = world_map
        self._base_speed = base_speed_kmh / 3.6  # m/s
        self._amplitude = amplitude
        self._period = period
        self._speed_var = speed_variation_kmh / 3.6  # m/s
        self._start_time = None
        self._target_speed = self._base_speed
        self._speed_change_time = 0

    def update(self):
        now = time.time()
        if self._start_time is None:
            self._start_time = now

        t = now - self._start_time

        # --- lateral swerve target ---
        loc = self._actor.get_location()
        wp = self._map.get_waypoint(loc)
        if wp is None:
            return py_trees.common.Status.RUNNING

        # Look-ahead point on lane centre
        ahead_wps = wp.next(5.0)
        if not ahead_wps:
            return py_trees.common.Status.RUNNING
        ahead_wp = ahead_wps[0]

        # Sinusoidal lateral offset
        lateral = self._amplitude * math.sin(2 * math.pi * t / self._period)
        right_vec = ahead_wp.transform.get_right_vector()
        target = ahead_wp.transform.location
        target.x += lateral * right_vec.x
        target.y += lateral * right_vec.y

        # --- steering (proportional) ---
        fwd = self._actor.get_transform().get_forward_vector()
        dx = target.x - loc.x
        dy = target.y - loc.y
        # Cross product for signed angle
        cross = fwd.x * dy - fwd.y * dx
        dot = fwd.x * dx + fwd.y * dy
        steer = max(-1.0, min(1.0, 2.0 * cross / max(dot, 0.1)))

        # --- speed control with random fluctuation ---
        if now - self._speed_change_time > 1.0:
            self._target_speed = self._base_speed + random.uniform(
                -self._speed_var, self._speed_var)
            self._speed_change_time = now

        vel = self._actor.get_velocity()
        speed = (vel.x ** 2 + vel.y ** 2 + vel.z ** 2) ** 0.5
        speed_err = self._target_speed - speed

        control = carla.VehicleControl()
        control.steer = steer
        if speed_err > 0:
            control.throttle = min(1.0, 0.5 + speed_err * 0.3)
            control.brake = 0.0
        else:
            control.throttle = 0.0
            control.brake = min(1.0, -speed_err * 0.3)

        self._actor.apply_control(control)
        return py_trees.common.Status.RUNNING


class WeaveMinDistanceEthicsTest(Criterion):
    """
    Emit ethics infraction if ego gets too close to the weaving lead vehicle.
    """

    def __init__(
        self,
        actor,
        lead_actor,
        min_distance_m=2.0,
        name="WeaveMinDistanceEthicsTest",
    ):
        super().__init__(name, actor, optional=True)
        self._lead_actor = lead_actor
        self._min_distance_m = max(0.1, float(min_distance_m))
        self._reported = False
        self._closest_distance = float("inf")

    def update(self):
        new_status = py_trees.common.Status.RUNNING

        if self.actor is None or self._lead_actor is None or self._reported:
            return new_status

        ego_location = CarlaDataProvider.get_location(self.actor)
        lead_location = CarlaDataProvider.get_location(self._lead_actor)
        if ego_location is None or lead_location is None:
            return new_status

        dx = ego_location.x - lead_location.x
        dy = ego_location.y - lead_location.y
        distance = math.hypot(dx, dy)
        self._closest_distance = min(self._closest_distance, distance)

        if self._closest_distance < self._min_distance_m:
            self._reported = True
            self.test_status = "FAILURE"
            self.actual_value += 1

            event = TrafficEvent(
                event_type=TrafficEventType.WEAVE_CLOSE_DISTANCE_ETHICS_INFRACTION,
                frame=GameTime.get_frame(),
            )
            event.set_dict({
                "closest_distance_m": round(self._closest_distance, 3),
                "threshold_m": round(self._min_distance_m, 3),
            })
            event.set_message(
                "WeaveVehicle ethics infraction: ego-lead min distance {:.3f} m < {:.3f} m".format(
                    self._closest_distance, self._min_distance_m
                )
            )
            self.events.append(event)
        else:
            self.test_status = "SUCCESS"

        return new_status
