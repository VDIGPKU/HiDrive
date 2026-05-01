#!/usr/bin/env python

# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Scenarios in which another (opposite) vehicle 'illegally' takes
priority, e.g. by running a red traffic light.
"""

from __future__ import print_function

import py_trees
import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (ActorDestroy,
                                                                      ActorTransformSetter,
                                                                      WaypointFollower,
                                                                      KeepVelocity,
                                                                      Idle)
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (InTriggerDistanceToVehicle,
                                                                                DriveDistance)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenarios.basic_scenario import BasicScenario

from srunner.tools.background_manager import HandleJunctionScenario

from srunner.tools.scenario_helper import generate_target_waypoint, generate_target_waypoint_list_multilane

class ActorAlignedWithEgo(py_trees.behaviour.Behaviour):
    """
    Check if actor's x position is aligned with or ahead of ego vehicle
    """
    def __init__(self, actor, ego_vehicle, name="ActorAlignedWithEgo"):
        super(ActorAlignedWithEgo, self).__init__(name)
        self._actor = actor
        self._ego = ego_vehicle

    def update(self):
        actor_x = CarlaDataProvider.get_location(self._actor).x
        ego_x = CarlaDataProvider.get_location(self._ego).x
        if actor_x >= ego_x - 5:  # Allow 10m tolerance (cut in earlier)
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING


class EgoRelativeSpeedWaypointFollower(WaypointFollower):
    """Waypoint follower whose target speed tracks ego speed + delta (km/h)."""

    def __init__(self, actor, ego_vehicle, speed_delta_kmh=15.0, plan=None,
                 avoid_collision=False, name="EgoRelativeSpeedWaypointFollower"):
        super(EgoRelativeSpeedWaypointFollower, self).__init__(
            actor,
            target_speed=0.0,
            plan=plan,
            avoid_collision=avoid_collision,
            name=name
        )
        self._ego_vehicle = ego_vehicle
        self._speed_delta_kmh = float(speed_delta_kmh)

    def update(self):
        ego_speed_mps = CarlaDataProvider.get_velocity(self._ego_vehicle) or 0.0
        target_speed_kmh = max(0.0, ego_speed_mps * 3.6 + self._speed_delta_kmh)

        for actor, local_planner in self._local_planner_dict.items():
            if actor is None or not actor.is_alive or local_planner is None:
                continue
            if hasattr(local_planner, "set_speed"):
                local_planner.set_speed(target_speed_kmh)

        return super(EgoRelativeSpeedWaypointFollower, self).update()


def convert_dict_to_location(actor_dict):
    """
    Convert a JSON string to a Carla.Location
    """
    location = carla.Location(
        x=float(actor_dict['x']),
        y=float(actor_dict['y']),
        z=float(actor_dict['z'])
    )
    return location

class HighwayCutIn(BasicScenario):
    """
    This class holds everything required for a scenario in which another vehicle runs a red light
    in front of the ego, forcing it to react. This vehicles are 'special' ones such as police cars,
    ambulances or firetrucks.
    """

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=180):
        """
        Setup all relevant parameters and create scenario
        and instantiate scenario manager
        """
        self._world = world
        self._map = CarlaDataProvider.get_map()
        self.timeout = timeout

        self._same_lane_time = 0.3
        self._other_lane_time = 3
        self._change_time = 2
        self._cut_in_distance = 10
        self._extra_space = 170

        self._start_location = convert_dict_to_location(config.other_parameters['other_actor_location'])
        self._cut_in_direction = config.other_parameters.get('cut_in_direction', {}).get('value', 'left')
        # XML configurable: distance (meters) used to complete the lane-change cut-in maneuver.
        raw_cut_in_completion = config.other_parameters.get(
            'cut_in_completion_distance',
            {}
        ).get(
            'value',
            config.other_parameters.get('total_lane_change_distance', {}).get('value', 3.0)
        )
        try:
            self._cut_in_completion_distance = max(0.5, float(raw_cut_in_completion))
        except (TypeError, ValueError):
            self._cut_in_completion_distance = 3.0

        super().__init__("HighwayCutIn",
                         ego_vehicles,
                         config,
                         world,
                         debug_mode,
                         criteria_enable=criteria_enable)

    def _initialize_actors(self, config):
        """
        Custom initialization
        """
        self._other_waypoint = self._map.get_waypoint(self._start_location)
        self._other_transform = self._other_waypoint.transform

        self._cut_in_vehicle = CarlaDataProvider.request_new_actor(
            'vehicle.*', self._other_transform, rolename='scenario',
            attribute_filter={'base_type': 'car', 'has_lights': True}
        )
        self.other_actors.append(self._cut_in_vehicle)

        # Place vehicle at start position immediately (visible from scenario start)
        self._cut_in_vehicle.set_transform(self._other_transform)
        self._cut_in_vehicle.set_simulate_physics(True)


    def _create_behavior(self):
        """
        Cut-in scenario: other vehicle accelerates and cuts in very close to ego
        """
        behavior = py_trees.composites.Sequence("HighwayCutIn")

        if self.route_mode:
            behavior.add_child(HandleJunctionScenario(
                clear_junction=True,
                clear_ego_entry=False,
                remove_entries=[self._other_waypoint],
                remove_exits=[],
                stop_entries=False,
                extend_road_exit=self._extra_space
            ))

        # Drive fast on same lane until close to ego
        same_lane_plan = []
        wp = self._other_waypoint
        for _ in range(30):  # Reduce to 30m path
            same_lane_plan.append((wp, 1))
            next_wps = wp.next(2.0)  # Increase step to 2m
            if not next_wps:
                break
            wp = next_wps[0]

        drive_until_close = py_trees.composites.Parallel(
            "DriveUntilClose",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        drive_until_close.add_child(EgoRelativeSpeedWaypointFollower(
            self._cut_in_vehicle,
            self.ego_vehicles[0],
            speed_delta_kmh=20.0,
            plan=same_lane_plan,
            name="CutInRelativeSpeedFollow"
        ))
        drive_until_close.add_child(ActorAlignedWithEgo(
            self._cut_in_vehicle, self.ego_vehicles[0]))
        behavior.add_child(drive_until_close)

        # Generate lane change path from estimated position
        estimated_wp = self._other_waypoint.next(30)[0] if self._other_waypoint.next(30) else self._other_waypoint
        plan, _ = generate_target_waypoint_list_multilane(
            estimated_wp, change='right',
            distance_same_lane=0, distance_other_lane=120,
            total_lane_change_distance=self._cut_in_completion_distance, check=False)

        # Follow the lane change path at lower speed to create braking scenario
        if plan:
            behavior.add_child(WaypointFollower(
                self._cut_in_vehicle, 10, plan=plan))

        behavior.add_child(ActorDestroy(self._cut_in_vehicle))
        return behavior

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        if self.route_mode:
            return []
        return [CollisionTest(self.ego_vehicles[0])]

    def __del__(self):
        """
        Remove all actors and traffic lights upon deletion
        """
        self.remove_all_actors()
