#!/usr/bin/env python

# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Rear-end pause scenario:
- Two vehicles are spawned ahead in ego lane.
- Rear vehicle drives forward and rear-ends the front vehicle.
- After impact/close contact they both remain stopped for a while.

XML parameters (all optional):
- front_distance (m): front vehicle distance ahead of trigger, default 34
- rear_distance (m): rear vehicle distance ahead of trigger, default 24
- rear_speed_kmh (km/h): approach speed of rear vehicle, default 24
- impact_trigger_distance (m): distance to treat as "rear-end happened", default 2.0
- impact_travel_distance (m): max distance rear vehicle drives before forced stop, default 16
- stop_hold_time (s): keep both vehicles stopped after rear-end, default 60
- front_vehicle_model: blueprint id/filter, default "vehicle.ue4.chevrolet.impala"
- rear_vehicle_model: blueprint id/filter, default "vehicle.dodgecop.charger"
- pedestrian_count: number of static pedestrians around crash, default 0
- pedestrian_model: walker blueprint filter, default "walker.pedestrian.*"
- pedestrian_lateral_offset (m): side distance from lane center, default 2.5
- pedestrian_longitudinal_step (m): spacing along lane for multiple pedestrians, default 2.0
- pedestrian_z_offset (m): vertical offset from sampled ground for static walkers, default 1.0
- static_collision_layout: if true, skip rear-end motion and spawn two stopped vehicles bumper-to-bumper
- static_vehicle_gap (m): rear vehicle spawn gap behind front in static layout, default 0.8
"""

from __future__ import print_function

import carla
import py_trees

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (
    ActorDestroy,
    ActorTransformSetter,
    HandBrakeVehicle,
    Idle,
    StopVehicle,
    WaypointFollower,
)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (
    DriveDistance,
    InTriggerDistanceToVehicle,
)
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.tools.scenario_helper import get_waypoint_in_distance


def _get_param(config, name, cast, default):
    if hasattr(config, "other_parameters") and name in config.other_parameters:
        try:
            return cast(config.other_parameters[name].get("value", default))
        except (TypeError, ValueError):
            return default
    return default


def _get_bool_param(config, name, default=False):
    if not hasattr(config, "other_parameters") or name not in config.other_parameters:
        return default
    raw = str(config.other_parameters[name].get("value", default)).strip().lower()
    if raw in ("1", "true", "yes", "y", "on"):
        return True
    if raw in ("0", "false", "no", "n", "off"):
        return False
    return default


class RearEndPause(BasicScenario):
    """Two front vehicles rear-end and stop."""

    def __init__(self, world, ego_vehicles, config, debug_mode=False, criteria_enable=True, timeout=120):
        self._world = world
        self._map = CarlaDataProvider.get_map()
        self.timeout = timeout

        self._reference_waypoint = self._map.get_waypoint(config.trigger_points[0].location)
        self._front_distance = max(12.0, _get_param(config, "front_distance", float, 34.0))
        self._rear_distance = max(6.0, _get_param(config, "rear_distance", float, 24.0))
        self._static_collision_layout = _get_bool_param(config, "static_collision_layout", False)
        self._static_vehicle_gap = max(0.3, _get_param(config, "static_vehicle_gap", float, 0.8))
        if not self._static_collision_layout and self._rear_distance >= self._front_distance - 2.0:
            self._rear_distance = max(6.0, self._front_distance - 8.0)

        self._rear_speed_kmh = max(5.0, _get_param(config, "rear_speed_kmh", float, 24.0))
        self._rear_speed_mps = self._rear_speed_kmh / 3.6
        self._impact_trigger_distance = max(0.8, _get_param(config, "impact_trigger_distance", float, 2.0))
        self._impact_travel_distance = max(4.0, _get_param(config, "impact_travel_distance", float, 16.0))
        self._stop_hold_time = max(1.0, _get_param(config, "stop_hold_time", float, 60.0))

        self._front_model = str(
            _get_param(config, "front_vehicle_model", str, "vehicle.ue4.chevrolet.impala")
        ).strip() or "vehicle.*"
        self._rear_model = str(
            _get_param(config, "rear_vehicle_model", str, "vehicle.dodgecop.charger")
        ).strip() or "vehicle.*"
        self._pedestrian_count = max(0, int(_get_param(config, "pedestrian_count", int, 0)))
        self._pedestrian_model = str(
            _get_param(config, "pedestrian_model", str, "walker.pedestrian.*")
        ).strip() or "walker.pedestrian.*"
        self._pedestrian_lateral_offset = max(0.5, _get_param(config, "pedestrian_lateral_offset", float, 2.5))
        self._pedestrian_longitudinal_step = max(
            0.5, _get_param(config, "pedestrian_longitudinal_step", float, 2.0)
        )
        self._pedestrian_z_offset = _get_param(config, "pedestrian_z_offset", float, 1.0)

        self._front_transform = None
        self._rear_transform = None
        self._pedestrian_transforms = []
        self._pedestrian_actors = []

        super().__init__(
            "RearEndPause",
            ego_vehicles,
            config,
            world,
            debug_mode,
            criteria_enable=criteria_enable,
        )

    def _resolve_bp(self, model_filter):
        bp_library = self._world.get_blueprint_library()
        candidates = bp_library.filter(model_filter)
        if candidates:
            return candidates[0].id
        fallback = bp_library.filter("vehicle.*")
        return fallback[0].id if fallback else None

    def _resolve_walker_bp(self, model_filter):
        bp_library = self._world.get_blueprint_library()
        candidates = bp_library.filter(model_filter)
        if candidates:
            return candidates[0].id
        fallback = bp_library.filter("walker.pedestrian.*")
        return fallback[0].id if fallback else None

    def _build_pedestrian_transforms(self):
        transforms = []
        if self._pedestrian_count <= 0:
            return transforms

        ref_loc = self._front_transform.location
        ref_rot = self._front_transform.rotation
        fwd = self._front_transform.get_forward_vector()
        right = self._front_transform.get_right_vector()

        for i in range(self._pedestrian_count):
            side = 1.0 if (i % 2 == 0) else -1.0
            row = i // 2
            longitudinal_offset = (row - 0.5) * self._pedestrian_longitudinal_step
            lateral_offset = side * self._pedestrian_lateral_offset
            base_loc = carla.Location(
                x=ref_loc.x + fwd.x * longitudinal_offset + right.x * lateral_offset,
                y=ref_loc.y + fwd.y * longitudinal_offset + right.y * lateral_offset,
                z=ref_loc.z,
            )

            # Place static walkers at a stable visual height: sample nearby ground first,
            # then add a configurable offset because physics is disabled for these actors.
            ground_z = ref_loc.z
            ground_wp = self._map.get_waypoint(base_loc, project_to_road=False)
            if ground_wp is None:
                ground_wp = self._map.get_waypoint(base_loc, project_to_road=True)
            if ground_wp is not None:
                ground_z = ground_wp.transform.location.z
            loc = carla.Location(x=base_loc.x, y=base_loc.y, z=ground_z + self._pedestrian_z_offset)
            rot = carla.Rotation(
                pitch=0.0,
                yaw=ref_rot.yaw + (180.0 if side > 0 else 0.0),
                roll=0.0,
            )
            transforms.append(carla.Transform(loc, rot))
        return transforms

    def _initialize_actors(self, config):
        front_wp, _ = get_waypoint_in_distance(self._reference_waypoint, self._front_distance)
        rear_wp, _ = get_waypoint_in_distance(self._reference_waypoint, self._rear_distance)

        self._front_transform = carla.Transform(front_wp.transform.location, front_wp.transform.rotation)
        # Always spawn rear actor at a safe separated location first.
        # In static collision layout, it will be teleported near the front actor in behavior init.
        self._rear_transform = carla.Transform(rear_wp.transform.location, rear_wp.transform.rotation)

        front_bp = self._resolve_bp(self._front_model)
        rear_bp = self._resolve_bp(self._rear_model)
        if not front_bp or not rear_bp:
            raise ValueError("RearEndPause: unable to resolve front/rear vehicle blueprint")

        front_actor = CarlaDataProvider.request_new_actor(front_bp, self._front_transform)
        rear_actor = CarlaDataProvider.request_new_actor(rear_bp, self._rear_transform)
        if front_actor is None or rear_actor is None:
            raise ValueError("RearEndPause: unable to spawn front/rear actors")

        for actor in (front_actor, rear_actor):
            actor.set_simulate_physics(False)
            hidden = actor.get_location()
            hidden.z -= 200.0
            actor.set_location(hidden)

        self.other_actors.append(front_actor)  # index 0: front
        self.other_actors.append(rear_actor)   # index 1: rear

        self._pedestrian_transforms = self._build_pedestrian_transforms()
        if self._pedestrian_transforms:
            ped_bp = self._resolve_walker_bp(self._pedestrian_model)
            if ped_bp:
                for ped_tf in self._pedestrian_transforms:
                    ped = CarlaDataProvider.request_new_actor(ped_bp, ped_tf)
                    if ped is None:
                        continue
                    ped.set_simulate_physics(False)
                    hidden = ped.get_location()
                    hidden.z -= 200.0
                    ped.set_location(hidden)
                    self._pedestrian_actors.append(ped)
                    self.other_actors.append(ped)

    def _create_behavior(self):
        behavior = py_trees.composites.Sequence(name="RearEndPause")
        front = self.other_actors[0]
        rear = self.other_actors[1]

        behavior.add_child(ActorTransformSetter(front, self._front_transform))
        if self._static_collision_layout:
            front_extent_x = max(0.5, getattr(front.bounding_box.extent, "x", 2.2))
            rear_extent_x = max(0.5, getattr(rear.bounding_box.extent, "x", 2.2))
            center_distance = front_extent_x + rear_extent_x + self._static_vehicle_gap
            fwd = self._front_transform.get_forward_vector()
            rear_loc = carla.Location(
                x=self._front_transform.location.x - fwd.x * center_distance,
                y=self._front_transform.location.y - fwd.y * center_distance,
                z=self._front_transform.location.z,
            )
            rear_static_tf = carla.Transform(rear_loc, self._front_transform.rotation)
            behavior.add_child(ActorTransformSetter(rear, rear_static_tf))
        else:
            behavior.add_child(ActorTransformSetter(rear, self._rear_transform))
        for ped_actor, ped_tf in zip(self._pedestrian_actors, self._pedestrian_transforms):
            behavior.add_child(ActorTransformSetter(ped_actor, ped_tf, physics=False))

        # Keep front car parked.
        behavior.add_child(HandBrakeVehicle(front, True))

        if self._static_collision_layout:
            # Static crash layout: vehicles are spawned bumper-to-bumper and stay parked.
            behavior.add_child(HandBrakeVehicle(rear, True))
        else:
            # Rear approaches front until close contact (or distance cap), then both stop.
            rear_approach = py_trees.composites.Parallel(
                name="RearApproachUntilImpact",
                policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE,
            )
            rear_approach.add_child(
                WaypointFollower(rear, target_speed=self._rear_speed_mps, avoid_collision=False, name="RearApproach")
            )
            rear_approach.add_child(
                InTriggerDistanceToVehicle(rear, front, self._impact_trigger_distance, name="RearImpactDistance")
            )
            rear_approach.add_child(
                DriveDistance(rear, self._impact_travel_distance, name="RearImpactDistanceCap")
            )
            behavior.add_child(rear_approach)

            behavior.add_child(StopVehicle(rear, 1.0))
            behavior.add_child(HandBrakeVehicle(rear, True))
            behavior.add_child(HandBrakeVehicle(front, True))
        behavior.add_child(Idle(self._stop_hold_time))

        for ped_actor in self._pedestrian_actors:
            behavior.add_child(ActorDestroy(ped_actor))
        behavior.add_child(ActorDestroy(rear))
        behavior.add_child(ActorDestroy(front))
        return behavior

    def _create_test_criteria(self):
        if self.route_mode:
            return []
        return [CollisionTest(self.ego_vehicles[0])]

    def __del__(self):
        self.remove_all_actors()
