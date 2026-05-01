#!/usr/bin/env python

"""
Parallel parking scenario:

Ego drives along the road, then must parallel-park on the shoulder between
two stationary obstacle vehicles.  The obstacles are placed at configurable
distances in front of and behind the parking target on the shoulder lane.

The scenario ends when the ego is near the parking target and has stopped.
"""

import py_trees
import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import Idle
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import \
    InTriggerDistanceToLocation
from srunner.scenarios.basic_scenario import BasicScenario


def _loc(d):
    return carla.Location(x=float(d['x']), y=float(d['y']), z=float(d['z']))


class ParallelParking(BasicScenario):
    """
    Two vehicles are parked on the shoulder with a gap between them.
    The ego must parallel-park into the gap.

    XML parameters:
        parking_target  - centre of the gap (shoulder location)
        front_distance  - distance from target to front obstacle (default 8)
        rear_distance   - distance from target to rear obstacle  (default 8)
    """

    def __init__(self, world, ego_vehicles, config, randomize=False,
                 debug_mode=False, criteria_enable=True, timeout=240):
        self._world = world
        self._map = CarlaDataProvider.get_map()
        self.timeout = timeout

        self._parking_location = _loc(config.other_parameters['parking_target'])
        self._front_distance = float(
            config.other_parameters.get('front_distance', {}).get('value', 8))
        self._rear_distance = float(
            config.other_parameters.get('rear_distance', {}).get('value', 8))

        super().__init__("ParallelParking",
                         ego_vehicles, config, world,
                         debug_mode, criteria_enable=criteria_enable)

    def _initialize_actors(self, config):
        parking_wp = self._map.get_waypoint(
            self._parking_location, lane_type=carla.LaneType.Any)
        if parking_wp is None:
            raise RuntimeError("Cannot find waypoint at parking target")

        self._parking_wp = parking_wp
        bp_filter = {'base_type': 'car'}

        # --- front obstacle ---
        front_wps = parking_wp.next(self._front_distance)
        if not front_wps:
            raise RuntimeError("Cannot find waypoint for front obstacle")
        front_transform = front_wps[0].transform
        front_transform.location.z += 0.5

        self._front_vehicle = CarlaDataProvider.request_new_actor(
            'vehicle.*', front_transform, rolename='scenario no lights',
            attribute_filter=bp_filter)
        if self._front_vehicle is None:
            raise RuntimeError("Could not spawn front obstacle vehicle")
        self._front_vehicle.apply_control(carla.VehicleControl(hand_brake=True))
        self.other_actors.append(self._front_vehicle)

        # --- rear obstacle ---
        rear_wps = parking_wp.previous(self._rear_distance)
        if not rear_wps:
            raise RuntimeError("Cannot find waypoint for rear obstacle")
        rear_transform = rear_wps[0].transform
        rear_transform.location.z += 0.5

        self._rear_vehicle = CarlaDataProvider.request_new_actor(
            'vehicle.*', rear_transform, rolename='scenario no lights',
            attribute_filter=bp_filter)
        if self._rear_vehicle is None:
            raise RuntimeError("Could not spawn rear obstacle vehicle")
        self._rear_vehicle.apply_control(carla.VehicleControl(hand_brake=True))
        self.other_actors.append(self._rear_vehicle)

        print(f"  [ParallelParking] Front obstacle at {front_transform.location}")
        print(f"  [ParallelParking] Rear  obstacle at {rear_transform.location}")
        print(f"  [ParallelParking] Parking target at {self._parking_location}")

    def _create_behavior(self):
        """
        End when ego is within 3m of parking target and nearly stopped,
        or on scenario timeout.
        """
        behavior = py_trees.composites.Sequence("ParallelParking")

        # Wait until ego is close to parking target AND stopped
        end_condition = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE,
            name="ParkingEndCondition")

        parked_check = py_trees.composites.Sequence("ParkedCheck")
        parked_check.add_child(InTriggerDistanceToLocation(
            self.ego_vehicles[0], self._parking_location, 3.0))
        parked_check.add_child(EgoStopped(self.ego_vehicles[0]))

        end_condition.add_child(parked_check)
        end_condition.add_child(Idle(self.timeout))

        behavior.add_child(end_condition)
        return behavior

    def _create_test_criteria(self):
        if self.route_mode:
            return []
        return [CollisionTest(self.ego_vehicles[0])]

    def __del__(self):
        self.remove_all_actors()


class EgoStopped(py_trees.behaviour.Behaviour):
    """Succeed when the actor speed is below threshold for a few ticks."""

    def __init__(self, actor, speed_threshold=0.5, stable_ticks=20,
                 name="EgoStopped"):
        super().__init__(name)
        self._actor = actor
        self._speed_threshold = speed_threshold
        self._stable_ticks = stable_ticks
        self._count = 0

    def update(self):
        vel = self._actor.get_velocity()
        speed = (vel.x ** 2 + vel.y ** 2 + vel.z ** 2) ** 0.5
        if speed < self._speed_threshold:
            self._count += 1
        else:
            self._count = 0
        if self._count >= self._stable_ticks:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING
