#!/usr/bin/env python

# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Wrong-way vehicle scenario on a straight road segment.

Scenario behavior:
- Spawn one adversary vehicle ahead on ego lane, facing opposite direction.
- Adversary drives toward ego (wrong-way) with configurable speed.
- Background traffic can be disabled from route XML via <no_background_traffic value="true" />.

XML parameters (all optional):
- spawn_distance (m): distance ahead of trigger to spawn adversary, default 28
- oncoming_speed_kmh (km/h): wrong-way vehicle speed, default 25
- oncoming_speed (km/h): backward-compatible alias of oncoming_speed_kmh
- oncoming_travel_distance (m): planned travel distance toward ego, default 90
- oncoming_vehicle_models: comma-separated blueprint filters/ids (highest priority)
- oncoming_vehicle_model: single blueprint filter/id (backward-compatible)
"""

from __future__ import print_function

import carla
import py_trees
from agents.navigation.local_planner import RoadOption

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (
    ActorDestroy,
    ActorTransformSetter,
    WaypointFollower,
)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import DriveDistance
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.tools.scenario_helper import get_waypoint_in_distance


def _get_param(config, name, cast, default):
    if hasattr(config, "other_parameters") and name in config.other_parameters:
        try:
            return cast(config.other_parameters[name].get("value", default))
        except (TypeError, ValueError):
            return default
    return default


class WrongWayVehicle(BasicScenario):
    """
    One wrong-way vehicle drives toward ego on the ego lane.
    """

    def __init__(self, world, ego_vehicles, config, debug_mode=False, criteria_enable=True, timeout=120):
        self._world = world
        self._map = CarlaDataProvider.get_map()
        self.timeout = timeout

        self._reference_waypoint = self._map.get_waypoint(config.trigger_points[0].location)
        self._spawn_distance_m = _get_param(config, "spawn_distance", float, 28.0)
        self._oncoming_speed_kmh = _get_param(config, "oncoming_speed_kmh", float, 25.0)
        if self._oncoming_speed_kmh <= 0:
            self._oncoming_speed_kmh = _get_param(config, "oncoming_speed", float, 25.0)
        self._oncoming_speed_mps = max(1.0, self._oncoming_speed_kmh / 3.6)
        self._travel_distance_m = max(20.0, _get_param(config, "oncoming_travel_distance", float, 90.0))
        raw_models = str(_get_param(config, "oncoming_vehicle_models", str, "")).strip()
        raw_model_single = str(_get_param(config, "oncoming_vehicle_model", str, "")).strip()
        if raw_models:
            self._oncoming_vehicle_patterns = [x.strip() for x in raw_models.split(",") if x.strip()]
        elif raw_model_single:
            self._oncoming_vehicle_patterns = [x.strip() for x in raw_model_single.split(",") if x.strip()]
        else:
            self._oncoming_vehicle_patterns = ["vehicle.*"]

        self._spawn_transform = None
        self._plan = []
        self._plan_step_m = 2.0

        super().__init__(
            "WrongWayVehicle",
            ego_vehicles,
            config,
            world,
            debug_mode,
            criteria_enable=criteria_enable,
        )

    def _build_wrong_way_plan(self, start_wp):
        plan = [(start_wp, RoadOption.LANEFOLLOW)]
        wp = start_wp
        distance = 0.0

        while distance < self._travel_distance_m:
            prev_wps = wp.previous(self._plan_step_m)
            if not prev_wps:
                break
            prev_wp = prev_wps[0]
            step = prev_wp.transform.location.distance(wp.transform.location)
            distance += step
            plan.append((prev_wp, RoadOption.LANEFOLLOW))
            wp = prev_wp

        return plan

    def _resolve_oncoming_model_id(self):
        bp_library = self._world.get_blueprint_library()
        for pattern in self._oncoming_vehicle_patterns:
            candidates = bp_library.filter(pattern)
            if candidates:
                return candidates[0].id
        fallback = bp_library.filter("vehicle.*")
        if fallback:
            return fallback[0].id
        return None

    def _initialize_actors(self, config):
        spawn_wp, _ = get_waypoint_in_distance(self._reference_waypoint, self._spawn_distance_m)
        spawn_transform = carla.Transform(spawn_wp.transform.location, spawn_wp.transform.rotation)
        spawn_transform.rotation.yaw += 180.0  # face ego (wrong-way)
        self._spawn_transform = spawn_transform

        bp_id = self._resolve_oncoming_model_id()
        if not bp_id:
            raise ValueError("WrongWayVehicle: unable to resolve oncoming vehicle blueprint")

        actor = CarlaDataProvider.request_new_actor(bp_id, self._spawn_transform)
        if actor is None:
            actor = CarlaDataProvider.request_new_actor("vehicle.*", self._spawn_transform)
        if actor is None:
            raise ValueError("WrongWayVehicle: unable to spawn oncoming actor")

        actor.set_simulate_physics(False)
        hidden_loc = actor.get_location()
        hidden_loc.z -= 200.0
        actor.set_location(hidden_loc)
        self.other_actors.append(actor)

        self._plan = self._build_wrong_way_plan(spawn_wp)

    def _create_behavior(self):
        sequence = py_trees.composites.Sequence(name="WrongWayVehicle")
        adversary = self.other_actors[0]

        sequence.add_child(ActorTransformSetter(adversary, self._spawn_transform))

        drive = py_trees.composites.Parallel(
            name="WrongWayVehicleDrive",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE,
        )
        if len(self._plan) >= 2:
            drive.add_child(
                WaypointFollower(
                    adversary,
                    target_speed=self._oncoming_speed_mps,
                    plan=self._plan,
                    avoid_collision=False,
                    name="WrongWayFollower",
                )
            )
        else:
            drive.add_child(
                WaypointFollower(
                    adversary,
                    target_speed=self._oncoming_speed_mps,
                    avoid_collision=False,
                    name="WrongWayFollowerFallback",
                )
            )
        drive.add_child(DriveDistance(adversary, self._travel_distance_m))
        sequence.add_child(drive)
        sequence.add_child(ActorDestroy(adversary))
        return sequence

    def _create_test_criteria(self):
        if self.route_mode:
            return []
        return [CollisionTest(self.ego_vehicles[0])]

    def __del__(self):
        self.remove_all_actors()
