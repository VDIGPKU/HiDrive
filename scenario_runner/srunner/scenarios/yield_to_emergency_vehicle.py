#!/usr/bin/env python

# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Scenario in which the ego has to yield its lane to emergency vehicle.
"""

from __future__ import print_function

import py_trees
import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (ActorTransformSetter,
                                                                      ActorDestroy,
                                                                      Idle,
                                                                      AdaptiveConstantVelocityAgentBehavior,
                                                                      WaypointFollower)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest, YieldToEmergencyVehicleTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (InTriggerDistanceToVehicle,
                                                                               WaitUntilInFront,
                                                                               DriveDistance)
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.tools.background_manager import RemoveRoadLane, ReAddRoadLane


class YieldToEmergencyVehicle(BasicScenario):
    """
    This class holds everything required for a scenario in which the ego has
    to yield its lane to an emergency vehicle.
    The background activity will be removed from the lane the emergency vehicle
    will pass through,
    and will be recreated once the scenario is over.

    Should be on the highway which is long enough and has no junctions.
    There should be at least two lanes on the highway.

    Optional XML parameters:
      distance           - rear spawn distance in meters (default 140)
      speed_increment    - EV speed increment over ego in km/h (default 25)
      trigger_distance   - trigger distance for pressure phase (default 50)
      ev_idle_time       - waiting time after EV reaches trigger distance (default 10)
      ev_vehicle_type    - emergency / auto / ambulance / police / ambulance_or_police
      ev_vehicle_model   - exact blueprint id (optional)
      ev_vehicle_models  - comma-separated blueprint ids/patterns (optional)
      target_speed_kmh   - EV fixed target speed, km/h (optional, default disabled)
      no_slowdown        - true: use constant-speed follower without collision avoidance
    """

    _FIRETRUCK_PATTERNS = [
        "vehicle.firetruck.actors",
        "vehicle.carlamotors.firetruck",
        "vehicle.*firetruck*",
        "vehicle.*fire*truck*",
    ]
    _AMBULANCE_PATTERNS = [
        "vehicle.ambulance.ford",
        "vehicle.ford.ambulance",
        "vehicle.*ambulance*",
        "vehicle.*rescue*",
    ]
    _POLICE_PATTERNS = [
        "vehicle.dodge.charger_police_2020",
        "vehicle.dodge.charger_police",
        "vehicle.*police*",
    ]
    _EMERGENCY_KEYWORDS = ("firetruck", "ambulance", "police", "rescue")
    _SPAWN_DISTANCE_OFFSETS = [0, 5, -5, 10, -10, 15, -15, 20, -20, 30, -30, 40]
    _MODEL_ALIASES = {
        # Common id aliases between CARLA distributions
        "vehicle.ford.ambulance": ["vehicle.ambulance.ford"],
        "vehicle.dodge.charger_police_2020": ["vehicle.dodgecop.charger"],
        "vehicle.dodge.charger_police": ["vehicle.dodgecop.charger"],
        # Some packs use "cop" instead of "police"
        "vehicle.*police*": ["vehicle.*cop*"],
    }

    def __init__(self, world, ego_vehicles, config, debug_mode=False, criteria_enable=True,
                 timeout=180):
        """
        Setup all relevant parameters and create scenario
        and instantiate scenario manager
        """
        self._world = world
        self._map = CarlaDataProvider.get_map()
        self.timeout = timeout

        p = config.other_parameters

        self._distance = float(p.get('distance', {}).get('value', 140))
        # km/h. How much the EV is expected to be faster than the EGO
        self._speed_increment = float(p.get('speed_increment', {}).get('value', 25))
        self._trigger_distance = float(p.get('trigger_distance', {}).get('value', 50))
        self._ev_idle_time = float(p.get('ev_idle_time', {}).get('value', 10))
        self._target_speed_kmh = float(p.get('target_speed_kmh', {}).get('value', -1))
        self._no_slowdown = str(p.get('no_slowdown', {}).get('value', 'false')).strip().lower() in (
            '1', 'true', 'yes', 'on'
        )

        self._ev_vehicle_type = str(p.get('ev_vehicle_type', {}).get('value', 'emergency')).strip().lower()
        self._ev_vehicle_model = str(p.get('ev_vehicle_model', {}).get('value', '')).strip()
        self._ev_vehicle_models = str(p.get('ev_vehicle_models', {}).get('value', '')).strip()

        # Change some of the parameters to adapt its behavior.
        # 1) ConstantVelocityAgent = infinite acceleration -> reduce the detection radius to pressure the ego
        # 2) Always use the bb check to ensure the EV doesn't run over the ego when it is lane changing
        # 3) Add more wps to improve BB detection
        self._opt_dict = {
            'base_vehicle_threshold': 10, 'detection_speed_ratio': 0.15, 'use_bbs_detection': True,
            'base_min_distance': 1, 'distance_ratio': 0.2
            }

        self._trigger_location = config.trigger_points[0].location
        self._reference_waypoint = self._map.get_waypoint(self._trigger_location)

        self._end_distance = 50

        super().__init__("YieldToEmergencyVehicle",
                         ego_vehicles,
                         config,
                         world,
                         debug_mode,
                         criteria_enable=criteria_enable)

    @staticmethod
    def _split_csv(text):
        return [x.strip() for x in text.split(',') if x.strip()]

    def _get_ev_model_patterns(self):
        if self._ev_vehicle_models:
            return self._split_csv(self._ev_vehicle_models)
        if self._ev_vehicle_model:
            return [self._ev_vehicle_model]

        if self._ev_vehicle_type == 'ambulance':
            return self._AMBULANCE_PATTERNS + self._POLICE_PATTERNS + self._FIRETRUCK_PATTERNS
        if self._ev_vehicle_type in ('police', 'policecar'):
            return self._POLICE_PATTERNS + self._AMBULANCE_PATTERNS + self._FIRETRUCK_PATTERNS
        if self._ev_vehicle_type in ('ambulance_or_police', 'ambulance_police', 'rescue'):
            return self._AMBULANCE_PATTERNS + self._POLICE_PATTERNS + self._FIRETRUCK_PATTERNS
        if self._ev_vehicle_type == 'firetruck':
            return self._FIRETRUCK_PATTERNS + self._AMBULANCE_PATTERNS + self._POLICE_PATTERNS

        # auto / emergency / any / unknown
        return self._AMBULANCE_PATTERNS + self._POLICE_PATTERNS + self._FIRETRUCK_PATTERNS

    def _expand_model_aliases(self, pattern):
        alias = self._MODEL_ALIASES.get(pattern)
        if alias:
            return [pattern] + alias

        # Best-effort compatibility: if a custom pattern uses "police",
        # also try a "cop" variant when the server uses that naming scheme.
        if "police" in pattern:
            return [pattern, pattern.replace("police", "cop")]
        return [pattern]

    def _resolve_ev_blueprint_ids(self):
        bp_library = self._world.get_blueprint_library()
        model_patterns = self._get_ev_model_patterns()

        resolved_ids = []
        seen_ids = set()
        for pattern in model_patterns:
            pattern_matched = False
            for expanded_pattern in self._expand_model_aliases(pattern):
                matched_ids = sorted({bp.id for bp in bp_library.filter(expanded_pattern)})
                if matched_ids:
                    pattern_matched = True
                for bp_id in matched_ids:
                    if bp_id in seen_ids:
                        continue
                    resolved_ids.append(bp_id)
                    seen_ids.add(bp_id)
            if not pattern_matched:
                print("[YieldToEmergencyVehicle] WARN: no blueprint matched pattern '{}'".format(pattern))

        if resolved_ids:
            return resolved_ids

        # Last-resort fallback by id keyword matching
        fallback_ids = []
        for bp in bp_library.filter('vehicle.*'):
            bp_id = bp.id
            lower_id = bp_id.lower()
            if any(k in lower_id for k in self._EMERGENCY_KEYWORDS) and bp_id not in seen_ids:
                fallback_ids.append(bp_id)
                seen_ids.add(bp_id)

        return sorted(fallback_ids)

    def _get_ev_start_transform_candidates(self):
        candidates = []
        seen = set()

        for offset in self._SPAWN_DISTANCE_OFFSETS:
            distance = max(8.0, self._distance + offset)
            ev_points = self._reference_waypoint.previous(distance)
            if not ev_points:
                continue

            transform = ev_points[0].transform
            key = (
                round(transform.location.x, 2),
                round(transform.location.y, 2),
                round(transform.location.z, 2),
                round(transform.rotation.yaw, 1),
            )
            if key in seen:
                continue
            seen.add(key)
            candidates.append((distance, transform))

        return candidates

    def _initialize_actors(self, config):
        """
        Custom initialization
        """
        # Spawn emergency vehicle
        ev_blueprints = self._resolve_ev_blueprint_ids()
        spawn_candidates = self._get_ev_start_transform_candidates()
        if not spawn_candidates:
            raise ValueError("Couldn't find viable position for the emergency vehicle")

        actor = None
        selected_bp = None
        selected_distance = None

        # Priority 1: explicit blueprint candidates
        for bp_id in ev_blueprints:
            for distance, transform in spawn_candidates:
                actor = CarlaDataProvider.request_new_actor(bp_id, transform)
                if actor is not None:
                    self._ev_start_transform = transform
                    selected_bp = bp_id
                    selected_distance = distance
                    break
            if actor is not None:
                break

        # Priority 2: old special_type filter fallback (backward compatibility).
        # If user explicitly specified model(s), do not silently fallback to a different model.
        has_explicit_models = bool(self._ev_vehicle_model or self._ev_vehicle_models)
        if actor is None and not has_explicit_models:
            for distance, transform in spawn_candidates:
                actor = CarlaDataProvider.request_new_actor(
                    "vehicle.*.*", transform, attribute_filter={'special_type': 'emergency'})
                if actor is not None:
                    self._ev_start_transform = transform
                    selected_bp = "special_type:emergency"
                    selected_distance = distance
                    break

        if actor is None:
            raise Exception("Couldn't spawn the emergency vehicle")

        print("[YieldToEmergencyVehicle] EV blueprint: {}".format(selected_bp))
        print("[YieldToEmergencyVehicle] EV config: "
              "requested_distance={:.1f}m, used_distance={:.1f}m, "
              "speed_increment={:.1f}km/h, target_speed_kmh={:.1f}, no_slowdown={}".format(
                  self._distance, selected_distance, self._speed_increment,
                  self._target_speed_kmh, self._no_slowdown
              ))

        # Move the actor underground and remove its physics so that it doesn't fall
        actor.set_simulate_physics(False)
        new_location = actor.get_location()
        new_location.z -= 500
        actor.set_location(new_location)

        # Turn on special lights
        actor.set_light_state(carla.VehicleLightState(
            carla.VehicleLightState.Special1 | carla.VehicleLightState.Special2))

        self.other_actors.append(actor)

    def _create_behavior(self):
        """
        Spawn the EV behind and wait for it to be close-by. After it has approached,
        give the ego a certain amount of time to yield to it.
        
        Sequence:
        - RemoveRoadLane
        - ActorTransformSetter
        - Parallel:
            - AdaptiveConstantVelocityAgentBehavior
            - Sequence: (End condition 1)
                - InTriggerDistanceToVehicle:
                - Idle
            - Sequence: (End condition 2)
                - WaitUntilInFront
                - DriveDistance
        - ReAddRoadLane
        """
        sequence = py_trees.composites.Sequence(name="YieldToEmergencyVehicle")

        if self.route_mode:
            sequence.add_child(RemoveRoadLane(self._reference_waypoint))

        sequence.add_child(ActorTransformSetter(self.other_actors[0], self._ev_start_transform))

        main_behavior = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        end_condition_1 = py_trees.composites.Sequence()
        end_condition_1.add_child(InTriggerDistanceToVehicle(
            self.ego_vehicles[0], self.other_actors[0], self._trigger_distance))
        end_condition_1.add_child(Idle(self._ev_idle_time))

        end_condition_2 = py_trees.composites.Sequence()
        end_condition_2.add_child(WaitUntilInFront(self.other_actors[0], self.ego_vehicles[0]))
        end_condition_2.add_child(DriveDistance(self.other_actors[0], self._end_distance))

        main_behavior.add_child(end_condition_1)
        main_behavior.add_child(end_condition_2)

        if self._no_slowdown or self._target_speed_kmh > 0:
            if self._target_speed_kmh > 0:
                target_speed_mps = self._target_speed_kmh / 3.6
            else:
                # Preserve old increment semantics when fixed speed is not provided.
                target_speed_mps = max(
                    1.0,
                    CarlaDataProvider.get_velocity(self.ego_vehicles[0]) + self._speed_increment / 3.6
                )
            main_behavior.add_child(WaypointFollower(
                self.other_actors[0], target_speed=target_speed_mps, avoid_collision=False))
        else:
            main_behavior.add_child(AdaptiveConstantVelocityAgentBehavior(
                self.other_actors[0], self.ego_vehicles[0],
                speed_increment=self._speed_increment, opt_dict=self._opt_dict))

        sequence.add_child(main_behavior)

        sequence.add_child(ActorDestroy(self.other_actors[0]))

        if self.route_mode:
            sequence.add_child(ReAddRoadLane(0))

        return sequence

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criterias = []
        criterias.append(YieldToEmergencyVehicleTest(self.ego_vehicles[0], self.other_actors[0]))
        if not self.route_mode:
            criterias.append(CollisionTest(self.ego_vehicles[0]))

        return criterias

    def __del__(self):
        """
        Remove all actors and traffic lights upon deletion
        """
        self.remove_all_actors()
