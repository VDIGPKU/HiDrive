#!/usr/bin/env python

# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module provides Challenge routes as standalone scenarios
"""

from __future__ import print_function

import glob
import os
import re
import sys
import importlib
import inspect
import py_trees
import traceback
import numpy as np

import carla
from agents.navigation.local_planner import RoadOption

from srunner.scenarioconfigs.scenario_configuration import ActorConfigurationData
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider

from srunner.scenariomanager.scenarioatomics.atomic_behaviors import ScenarioTriggerer, Idle
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import WaitForBlackboardVariable
from srunner.scenariomanager.scenarioatomics.atomic_criteria import (CollisionTest,
                                                                     InRouteTest,
                                                                     RouteCompletionTest,
                                                                     OutsideRouteLanesTest,
                                                                     RunningRedLightTest,
                                                                     RunningStopTest,
                                                                     ActorBlockedTest,
                                                                     MinimumSpeedRouteTest,
                                                                     SpeedBumpOverspeedEthicsTest)

from srunner.scenarios.basic_scenario import BasicScenario
from srunner.scenarios.background_activity import BackgroundBehavior
from srunner.scenariomanager.weather_sim import RouteWeatherBehavior
from srunner.scenariomanager.lights_sim import RouteLightsBehavior
from srunner.scenariomanager.timer import RouteTimeoutBehavior

from leaderboard.utils.route_parser import RouteParser, DIST_THRESHOLD
from leaderboard.utils.route_manipulation import interpolate_trajectory

import leaderboard.utils.parked_vehicles as parked_vehicles

POLICE_FORCE_ROUTE_COMPLETION_BB_KEY = "PoliceInterceptStop_force_route_completion_100"
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
SPEED_BUMP_POINTS_PATH = os.environ.get(
    "HILEVAD_SPEED_BUMP_POINTS_PATH",
    os.path.join(REPO_ROOT, "leaderboard", "data", "speed_bump_points.txt"),
)
SPEED_BUMP_TRIGGER_RADIUS_M = 3.0
SPEED_BUMP_MAX_SPEED_KMH = 25.0
_SPEED_BUMP_CACHE = None


def _get_town_key(map_name):
    map_name = (map_name or "").lower()
    if "town10" in map_name:
        return "town10"
    if "town15" in map_name:
        return "town15"
    return None


def _extract_speed_bump_points(notes_path):
    points = {"town10": [], "town15": []}
    if not os.path.exists(notes_path):
        return points

    current_town = None
    coord_pattern = re.compile(
        r"[（(]\s*([+-]?\d+(?:\.\d+)?)\s*[,，]\s*([+-]?\d+(?:\.\d+)?)\s*[）)]"
    )
    with open(notes_path, "r", encoding="utf-8") as handle:
        for raw_line in handle:
            line = raw_line.strip()
            if not line:
                continue

            low = line.lower()
            if low.startswith("town10"):
                current_town = "town10"
                continue
            if low.startswith("town15"):
                current_town = "town15"
                continue

            if current_town is None:
                continue

            for match in coord_pattern.findall(line):
                x_val, y_val = float(match[0]), float(match[1])
                points[current_town].append((x_val, y_val))

    # De-duplicate while preserving order.
    for town_key in points:
        dedup = []
        seen = set()
        for coord in points[town_key]:
            rounded = (round(coord[0], 4), round(coord[1], 4))
            if rounded in seen:
                continue
            seen.add(rounded)
            dedup.append(coord)
        points[town_key] = dedup
    return points


def _get_speed_bump_points_for_map(map_name):
    global _SPEED_BUMP_CACHE
    if _SPEED_BUMP_CACHE is None:
        _SPEED_BUMP_CACHE = _extract_speed_bump_points(SPEED_BUMP_POINTS_PATH)

    town_key = _get_town_key(map_name)
    if town_key is None:
        return []
    return list(_SPEED_BUMP_CACHE.get(town_key, []))


class RouteScenario(BasicScenario):

    """
    Implementation of a RouteScenario, i.e. a scenario that consists of driving along a pre-defined route,
    along which several smaller scenarios are triggered
    """

    category = "RouteScenario"
    INIT_THRESHOLD = 500 # Runtime initialization trigger distance to ego (m)
    PARKED_VEHICLES_INIT_THRESHOLD = INIT_THRESHOLD - 50 # Runtime initialization trigger distance to parked vehicles (m)

    def __init__(self, world, config, debug_mode=0, criteria_enable=True):
        """
        Setup all relevant parameters and create scenarios along route
        """
        self.client = CarlaDataProvider.get_client()
        self.config = config
        self.route = self._get_route(config)
        self.world = world
        self.map = CarlaDataProvider.get_map()
        self.timeout = 10000

        self.all_scenario_classes = None
        self.ego_data = None

        self.scenario_triggerer = None
        self.behavior_node = None # behavior node created by _create_behavior()
        self.criteria_node = None # criteria node created by _create_test_criteria()

        self.list_scenarios = []
        self.occupied_parking_locations = []
        self.available_parking_locations = []

        scenario_configurations = self._filter_scenarios(config.scenario_configs)
        print(f"\n{'='*60}")
        print(f"  Scenario configuration:")
        print(f"  - Raw scenario count: {len(config.scenario_configs)}")
        print(f"  - Filtered scenario count: {len(scenario_configurations)}")
        for sc in scenario_configurations:
            print(f"    - {sc.name} ({sc.type}) @ {sc.trigger_points[0].location if sc.trigger_points else 'N/A'}")
        print(f"{'='*60}\n")
        self.scenario_configurations = scenario_configurations
        self.missing_scenario_configurations = scenario_configurations.copy()

        ego_vehicle = self._spawn_ego_vehicle()
        if ego_vehicle is None:
            raise ValueError("Shutting down, couldn't spawn the ego vehicle")

        if debug_mode>0:
            self._draw_waypoints(self.route, vertical_shift=0.1, size=0.1, downsample=10)

        self._parked_ids = []
        self._get_parking_slots()

        super(RouteScenario, self).__init__(
            config.name, [ego_vehicle], config, world, debug_mode > 3, False, criteria_enable
        )

        # Route mode skips _initialize_environment in BasicScenario.__init__,
        # so we call it explicitly here to apply weather settings
        self._initialize_environment(world)

        # Do it after the 'super', as we need the behavior and criteria tree to be initialized
        self.build_scenarios(ego_vehicle, debug=debug_mode > 0)

        # Set runtime init mode. Do this after the first set of scenarios has been initialized!
        CarlaDataProvider.set_runtime_init_mode(True)

    def _get_route(self, config):
        """
        Gets the route from the configuration, interpolating it to the desired density,
        saving it to the CarlaDataProvider and sending it to the agent

        Parameters:
        - world: CARLA world
        - config: Scenario configuration (RouteConfiguration)
        - debug_mode: boolean to decide whether or not the route poitns are printed
        """

        # Prepare route's trajectory (interpolate and add the GPS route)
        self.gps_route, self.route = interpolate_trajectory(config.keypoints)

        # Preserve the original first keypoint location so that ego can spawn
        # off the driving lane (e.g. roadside merge scenarios).  trace_route
        # snaps everything to driving-lane centres, which would move the ego.
        if config.keypoints:
            orig = config.keypoints[0]
            first_transform = carla.Transform(
                carla.Location(x=orig.x, y=orig.y, z=orig.z),
                self.route[0][0].rotation
            )
            self.route[0] = (first_transform, self.route[0][1])

        return self.route

    def _filter_scenarios(self, scenario_configs):
        """
        Given a list of scenarios, filters out does that don't make sense to be triggered,
        as they are either too far from the route or don't fit with the route shape

        Parameters:
        - scenario_configs: list of ScenarioConfiguration
        """
        new_scenarios_config = []
        print(f"\n[DEBUG] Filtering {len(scenario_configs)} scenario configurations")
        for scenario_number, scenario_config in enumerate(scenario_configs):
            if not scenario_config.trigger_points:
                print(f"[DEBUG] Scenario '{scenario_config.name}' has no trigger point; skipping")
                continue
            trigger_point = scenario_config.trigger_points[0]
            print(f"[DEBUG] Checking scenario '{scenario_config.name}' trigger point: {trigger_point.location}, yaw={trigger_point.rotation.yaw}")
            if not RouteParser.is_scenario_at_route(trigger_point, self.route):
                print(f"WARNING: Ignoring scenario '{scenario_config.name}' as it is too far from the route")
                continue

            scenario_config.route_var_name = "ScenarioRouteNumber{}".format(scenario_number)
            new_scenarios_config.append(scenario_config)
            print(f"[DEBUG] Scenario '{scenario_config.name}' added to scenario list")

        return new_scenarios_config

    def _spawn_ego_vehicle(self):
        """Spawn the ego vehicle at the first waypoint of the route"""
        elevate_transform = self.route[0][0]
        elevate_transform.location.z += 0.5

        blueprint_library = self.world.get_blueprint_library()

        preferred_vehicles = [
            'vehicle.tesla.model3',
            'vehicle.audi.a2',
            'vehicle.bmw.grandtourer',
            'vehicle.mini.cooper_s',
            'vehicle.audi.tt',
            'vehicle.mercedes.coupe',
            'vehicle.dodge.charger_2020',
            'vehicle.ford.mustang',
            'vehicle.lincoln.mkz_2020',
            'vehicle.toyota.prius',
            'vehicle.nissan.micra',
            'vehicle.seat.leon',
            'vehicle.citroen.c3',
        ]

        vehicle_bp = None
        for name in preferred_vehicles:
            bps = list(blueprint_library.filter(name))
            if len(bps) > 0:
                vehicle_bp = bps[0]
                print(f"Found vehicle blueprint: {vehicle_bp.id}")
                break

        if vehicle_bp is None:
            print("No preferred vehicle found, searching for a small vehicle...")
            excluded_keywords = [
                'firetruck', 'ambulance', 'truck', 'bus', 'van',
                'carlacola', 'cybertruck', 'sprinter', 'isetta'
            ]
            all_vehicles = list(blueprint_library.filter('vehicle.*'))
            for bp in all_vehicles:
                vehicle_id = bp.id.lower()
                if any(keyword in vehicle_id for keyword in excluded_keywords):
                    continue
                vehicle_bp = bp
                print(f"Using fallback vehicle: {vehicle_bp.id}")
                break

        if vehicle_bp is None:
            print("ERROR: No suitable vehicle blueprint found!")
            return None

        if vehicle_bp.has_attribute('role_name'):
            vehicle_bp.set_attribute('role_name', 'hero')

        ego_vehicle = self.world.try_spawn_actor(vehicle_bp, elevate_transform)
        if ego_vehicle is None:
            print(f"WARNING: Failed to spawn vehicle at {elevate_transform.location}")
            return None

        print(f"Spawned ego vehicle: {ego_vehicle.type_id}")

        CarlaDataProvider._carla_actor_pool[ego_vehicle.id] = ego_vehicle
        CarlaDataProvider.register_actor(ego_vehicle, elevate_transform)

        spectator = self.world.get_spectator()
        spectator.set_transform(carla.Transform(elevate_transform.location + carla.Location(z=50),
                                                    carla.Rotation(pitch=-90)))

        self.world.tick()

        return ego_vehicle

    def _get_parking_slots(self, max_distance=100, route_step=10):
        """Spawn parked vehicles."""

        def is_close(slot_location):
            for i in range(0, len(self.route), route_step):
                route_transform = self.route[i][0]
                if route_transform.location.distance(slot_location) < max_distance:
                    return True
            return False

        min_x, min_y = float('inf'), float('inf')
        max_x, max_y = float('-inf'), float('-inf')
        for route_transform, _ in self.route:
            min_x = min(min_x, route_transform.location.x - max_distance)
            min_y = min(min_y, route_transform.location.y - max_distance)
            max_x = max(max_x, route_transform.location.x + max_distance)
            max_y = max(max_y, route_transform.location.y + max_distance)

        # Occupied parking locations
        occupied_parking_locations = []
        for scenario in self.list_scenarios:
            occupied_parking_locations.extend(scenario.get_parking_slots())

        available_parking_locations = []
        map_name = self.map.name.split('/')[-1]
        available_parking_locations = getattr(parked_vehicles, map_name, [])

        # Exclude parking slots that are too far from the route
        for slot in available_parking_locations:
            slot_transform = carla.Transform(
                location=carla.Location(slot["location"][0], slot["location"][1], slot["location"][2]),
                rotation=carla.Rotation(slot["rotation"][0], slot["rotation"][1], slot["rotation"][2])
            )

            in_area = (min_x < slot_transform.location.x < max_x) and (min_y < slot_transform.location.y < max_y)
            close_to_route = is_close(slot_transform.location)
            if not in_area or not close_to_route:
                available_parking_locations.remove(slot)
                continue

        self.available_parking_locations = available_parking_locations

    def spawn_parked_vehicles(self, ego_vehicle, max_scenario_distance=10):
        """Spawn parked vehicles."""
        def is_close(slot_location, ego_location):
            return slot_location.distance(ego_location) < self.PARKED_VEHICLES_INIT_THRESHOLD
        def is_free(slot_location):
            for occupied_slot in self.occupied_parking_locations:
                if slot_location.distance(occupied_slot) < max_scenario_distance:
                    return False
            return True

        new_parked_vehicles = []

        ego_location = CarlaDataProvider.get_location(ego_vehicle)
        if ego_location is None:
            return

        for slot in self.available_parking_locations:
            slot_transform = carla.Transform(
                location=carla.Location(slot["location"][0], slot["location"][1], slot["location"][2]),
                rotation=carla.Rotation(slot["rotation"][0], slot["rotation"][1], slot["rotation"][2])
            )

            # Add all vehicles that are close to the ego and in a free space
            if is_close(slot_transform.location, ego_location) and is_free(slot_transform.location):
                mesh_bp = CarlaDataProvider.get_world().get_blueprint_library().filter("static.prop.mesh")[0]
                mesh_bp.set_attribute("mesh_path", slot["mesh"])
                mesh_bp.set_attribute("scale", "0.9")
                new_parked_vehicles.append(carla.command.SpawnActor(mesh_bp, slot_transform))
                self.available_parking_locations.remove(slot)

        # Add the actors to _parked_ids
        for response in CarlaDataProvider.get_client().apply_batch_sync(new_parked_vehicles):
            if not response.error:
                self._parked_ids.append(response.actor_id)

    # pylint: disable=no-self-use
    def _draw_waypoints(self, waypoints, vertical_shift, size, downsample=1):
        """
        Draw a list of waypoints at a certain height given in vertical_shift.
        """
        for i, w in enumerate(waypoints):
            if i % downsample != 0:
                continue

            wp = w[0].location + carla.Location(z=vertical_shift)

            if w[1] == RoadOption.LEFT:  # Yellow
                color = carla.Color(128, 128, 0)
            elif w[1] == RoadOption.RIGHT:  # Cyan
                color = carla.Color(0, 128, 128)
            elif w[1] == RoadOption.CHANGELANELEFT:  # Orange
                color = carla.Color(128, 32, 0)
            elif w[1] == RoadOption.CHANGELANERIGHT:  # Dark Cyan
                color = carla.Color(0, 32, 128)
            elif w[1] == RoadOption.STRAIGHT:  # Gray
                color = carla.Color(64, 64, 64)
            else:  # LANEFOLLOW
                color = carla.Color(0, 128, 0)  # Green

            self.world.debug.draw_point(wp, size=size, color=color, life_time=self.timeout)

        self.world.debug.draw_point(waypoints[0][0].location + carla.Location(z=vertical_shift), size=2*size,
                                    color=carla.Color(0, 0, 128), life_time=self.timeout)
        self.world.debug.draw_point(waypoints[-1][0].location + carla.Location(z=vertical_shift), size=2*size,
                                    color=carla.Color(128, 128, 128), life_time=self.timeout)

    def get_all_scenario_classes(self):
        """
        Searches through the 'scenarios' folder for all the Python classes
        """
        # Path of all scenario at "srunner/scenarios" folder
        scenarios_list = glob.glob("{}/srunner/scenarios/*.py".format(os.getenv('SCENARIO_RUNNER_ROOT', "./")))

        all_scenario_classes = {}

        for scenario_file in scenarios_list:

            # Get their module
            module_name = os.path.basename(scenario_file).split('.')[0]
            sys.path.insert(0, os.path.dirname(scenario_file))
            scenario_module = importlib.import_module(module_name)

            # And their members of type class
            for member in inspect.getmembers(scenario_module, inspect.isclass):
                # TODO: Filter out any class that isn't a child of BasicScenario
                all_scenario_classes[member[0]] = member[1]

        return all_scenario_classes

    def build_scenarios(self, ego_vehicle, debug=False):
        """
        Initializes the class of all the scenarios that will be present in the route.
        If a class fails to be initialized, a warning is printed but the route execution isn't stopped
        """
        new_scenarios = []

        if self.all_scenario_classes is None:
            self.all_scenario_classes = self.get_all_scenario_classes()
        if self.ego_data is None:
            self.ego_data = ActorConfigurationData(ego_vehicle.type_id, ego_vehicle.get_transform(), 'hero')

        # Part 1. Check all scenarios that haven't been initialized, starting them if close enough to the ego vehicle
        for scenario_config in self.missing_scenario_configurations:
            scenario_config.ego_vehicles = [self.ego_data]
            scenario_config.route = self.route

            try:
                scenario_class = self.all_scenario_classes[scenario_config.type]
                trigger_location = scenario_config.trigger_points[0].location

                ego_location = CarlaDataProvider.get_location(ego_vehicle)
                if ego_location is None:
                    continue

                # Only init scenarios that are close to ego
                if trigger_location.distance(ego_location) < self.INIT_THRESHOLD:
                    scenario_instance = scenario_class(self.world, [ego_vehicle], scenario_config, timeout=self.timeout)

                    # Add new scenarios to list
                    self.list_scenarios.append(scenario_instance)
                    new_scenarios.append(scenario_instance)
                    self.missing_scenario_configurations.remove(scenario_config)

                    self.occupied_parking_locations.extend(scenario_instance.get_parking_slots())

                    if debug:
                        scenario_loc = scenario_config.trigger_points[0].location
                        debug_loc = self.map.get_waypoint(scenario_loc).transform.location + carla.Location(z=0.2)
                        self.world.debug.draw_point(
                            debug_loc, size=0.2, color=carla.Color(128, 0, 0), life_time=self.timeout
                        )
                        self.world.debug.draw_string(
                            debug_loc, str(scenario_config.name), draw_shadow=False,
                            color=carla.Color(0, 0, 128), life_time=self.timeout, persistent_lines=True
                        )

            except Exception as e:
                print(f"\033[93mSkipping scenario '{scenario_config.name}' due to setup error: {e}")
                if debug:
                    print(f"\n{traceback.format_exc()}")
                print("\033[0m", end="")
                self.missing_scenario_configurations.remove(scenario_config)
                continue

        # Part 2. Add their behavior onto the route's behavior tree
        for scenario in new_scenarios:

            # Add behavior
            if scenario.behavior_tree is not None:
                self.behavior_node.add_child(scenario.behavior_tree)
                self.scenario_triggerer.add_blackboard(
                    [scenario.config.route_var_name, scenario.config.trigger_points[0].location]
                )

            # Add the criteria criteria
            scenario_criteria = scenario.get_criteria()
            if len(scenario_criteria) == 0:
                continue

            self.criteria_node.add_child(
                self._create_criterion_tree(scenario, scenario_criteria)
            )

    # pylint: enable=no-self-use
    def _initialize_actors(self, config):
        """
        Set other_actors to the superset of all scenario actors
        """
        # Add all the actors of the specific scenarios to self.other_actors
        for scenario in self.list_scenarios:
            self.other_actors.extend(scenario.other_actors)

    def _create_behavior(self):
        """
        Creates a parallel behavior that runs all of the scenarios part of the route.
        These subbehaviors have had a trigger condition added so that they wait until
        the agent is close to their trigger point before activating.

        It also adds the BackgroundActivity scenario, which will be active throughout the whole route.
        This behavior never ends and the end condition is given by the RouteCompletionTest criterion.
        """
        scenario_trigger_distance = DIST_THRESHOLD  # Max trigger distance between route and scenario

        behavior = py_trees.composites.Parallel(name="Route Behavior",
                                                policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)

        self.behavior_node = behavior
        scenario_behaviors = []
        blackboard_list = []

        # Add the behavior that manages the scenario trigger conditions
        scenario_triggerer = ScenarioTriggerer(
            self.ego_vehicles[0], self.route, blackboard_list, scenario_trigger_distance)
        behavior.add_child(scenario_triggerer)  # Tick the ScenarioTriggerer before the scenarios

        # register var
        self.scenario_triggerer = scenario_triggerer

        # Add the Background Activity (skip if any scenario has no_background_traffic)
        skip_bg = any(
            sc.other_parameters.get('no_background_traffic')
            for sc in self.scenario_configurations
        )
        if skip_bg:
            print("[RouteScenario] Background traffic DISABLED for this route")
        else:
            behavior.add_child(BackgroundBehavior(self.ego_vehicles[0], self.route, name="BackgroundActivity"))

        behavior.add_children(scenario_behaviors)
        return behavior

    def _create_test_criteria(self):
        """
        Create the criteria tree. It starts with some route criteria (which are always active),
        and adds the scenario specific ones, which will only be active during their scenario
        """
        criteria = py_trees.composites.Parallel(name="Criteria",
                                                policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        self.criteria_node = criteria

        # End condition
        criteria.add_child(RouteCompletionTest(self.ego_vehicles[0], route=self.route))
        # Early-finish hook for PoliceInterceptStop compliant pull-over.
        criteria.add_child(WaitForBlackboardVariable(
            POLICE_FORCE_ROUTE_COMPLETION_BB_KEY,
            True,
            False,
            name="PoliceForcedRouteCompletion",
        ))

        # 'Normal' criteria
        criteria.add_child(OutsideRouteLanesTest(self.ego_vehicles[0], route=self.route))
        criteria.add_child(CollisionTest(self.ego_vehicles[0], name="CollisionTest"))
        criteria.add_child(RunningRedLightTest(self.ego_vehicles[0]))
        criteria.add_child(RunningStopTest(self.ego_vehicles[0]))
        criteria.add_child(MinimumSpeedRouteTest(self.ego_vehicles[0], self.route, checkpoints=20, name="MinSpeedTest"))
        speed_bump_points = _get_speed_bump_points_for_map(getattr(self.map, "name", ""))
        if speed_bump_points:
            print(
                "[RouteScenario] Speed-bump ethics monitor enabled: {} points, radius={}m, speed_limit={}km/h".format(
                    len(speed_bump_points), SPEED_BUMP_TRIGGER_RADIUS_M, SPEED_BUMP_MAX_SPEED_KMH
                )
            )
            criteria.add_child(
                SpeedBumpOverspeedEthicsTest(
                    self.ego_vehicles[0],
                    speed_bump_points=speed_bump_points,
                    trigger_radius=SPEED_BUMP_TRIGGER_RADIUS_M,
                    speed_threshold_kmh=SPEED_BUMP_MAX_SPEED_KMH,
                )
            )

        # These stop the route early to save computational time
        criteria.add_child(InRouteTest(
            self.ego_vehicles[0], route=self.route, offroad_max=30, terminate_on_failure=True))
        criteria.add_child(ActorBlockedTest(
            self.ego_vehicles[0], min_speed=0.1, max_time=60.0, terminate_on_failure=True, name="AgentBlockedTest")
        )

        return criteria

    def _create_weather_behavior(self):
        """
        Create the weather behavior
        """
        if len(self.config.weather) == 1:
            return  # Just set the weather at the beginning and done
        return RouteWeatherBehavior(self.ego_vehicles[0], self.route, self.config.weather)

    def _create_lights_behavior(self):
        """
        Create the street lights behavior
        """
        return RouteLightsBehavior(self.ego_vehicles[0], 100)

    def _create_timeout_behavior(self):
        """
        Create the timeout behavior
        """
        return RouteTimeoutBehavior(self.ego_vehicles[0], self.route)

    def _initialize_environment(self, world):
        """
        Set the weather
        """
        # Set the appropriate weather conditions
        weather_params = self.config.weather[0][1]
        world.set_weather(weather_params)

        # Diagnostic: check if weather was applied
        is_enabled = world.is_weather_enabled()
        applied = world.get_weather()
        print(f"[Weather] is_weather_enabled: {is_enabled}")
        print(f"[Weather] precipitation={applied.precipitation}, "
              f"precipitation_deposits={applied.precipitation_deposits}, "
              f"wetness={applied.wetness}, cloudiness={applied.cloudiness}, "
              f"fog_density={applied.fog_density}, "
              f"sun_altitude_angle={applied.sun_altitude_angle}")
        if not is_enabled:
            print("[Weather] WARNING: Weather system is DISABLED in this UE5 build. "
                  "Post-process rain effects (M_screenDrops) may not work.")

    def _create_criterion_tree(self, scenario, criteria):
        """
        We can make use of the blackboard variables used by the behaviors themselves,
        as we already have an atomic that handles their (de)activation.
        The criteria will wait until that variable is active (the scenario has started),
        and will automatically stop when it deactivates (as the scenario has finished)
        """
        scenario_name = scenario.name
        var_name = scenario.config.route_var_name
        check_name = "WaitForBlackboardVariable: {}".format(var_name)

        criteria_tree = py_trees.composites.Sequence(name=scenario_name)
        criteria_tree.add_child(WaitForBlackboardVariable(var_name, True, False, name=check_name))

        scenario_criteria = py_trees.composites.Parallel(name=scenario_name,
                                                policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        for criterion in criteria:
            scenario_criteria.add_child(criterion)
        scenario_criteria.add_child(WaitForBlackboardVariable(var_name, False, None, name=check_name))

        criteria_tree.add_child(scenario_criteria)
        criteria_tree.add_child(Idle())  # Avoid the indiviual criteria stopping the simulation
        return criteria_tree

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.client.apply_batch([carla.command.DestroyActor(x) for x in self._parked_ids])
        self.remove_all_actors()
