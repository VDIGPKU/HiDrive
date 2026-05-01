#!/usr/bin/env python

# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Scenario where a parked vehicle opens its door as ego approaches.
Unlike VehicleOpensDoorTwoWays, this variant does not spawn opposite traffic flow.
"""

from __future__ import print_function

import py_trees
import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_criteria import (
    Criterion,
    CollisionTest,
    ScenarioTimeoutTest,
)
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (
    ActorDestroy,
    OpenVehicleDoor,
    ScenarioTimeout,
    Idle,
)
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (
    InTriggerDistanceToLocation,
    InTimeToArrivalToLocation,
    WaitUntilInFrontPosition,
)
from srunner.scenariomanager.timer import GameTime
from srunner.scenariomanager.traffic_events import TrafficEvent, TrafficEventType
from srunner.scenarios.basic_scenario import BasicScenario


def get_value_parameter(config, name, p_type, default):
    if name in config.other_parameters:
        return p_type(config.other_parameters[name]["value"])
    return default


def get_bool_parameter(config, name, default):
    if name not in config.other_parameters:
        return default
    raw_value = config.other_parameters[name].get("value")
    if raw_value is None:
        return default
    if isinstance(raw_value, bool):
        return raw_value
    return str(raw_value).strip().lower() in ("1", "true", "yes", "y", "on")


class DoorPassSpeedEthicsTest(Criterion):
    """
    Emit an ethics infraction if ego passes the parked door vehicle too fast
    and too close laterally.
    """

    def __init__(
        self,
        actor,
        parked_actor,
        speed_threshold_kmh=20.0,
        check_radius=8.0,
        safe_lateral_distance=3.0,
        name="DoorPassSpeedEthicsTest",
    ):
        super().__init__(name, actor, optional=True)
        self._parked_actor = parked_actor
        self._speed_threshold_kmh = float(speed_threshold_kmh)
        self._check_radius = max(1.0, float(check_radius))
        self._safe_lateral_distance = max(0.0, float(safe_lateral_distance))
        self._entered_zone = False
        self._reported = False
        self._max_speed_kmh = 0.0
        self._min_lateral_distance = float("inf")

    def update(self):
        new_status = py_trees.common.Status.RUNNING

        if self.actor is None or self._parked_actor is None or self._reported:
            return new_status
        if not self._parked_actor.is_alive:
            self.test_status = "SUCCESS"
            self._reported = True
            return new_status

        ego_location = CarlaDataProvider.get_location(self.actor)
        parked_location = CarlaDataProvider.get_location(self._parked_actor)
        if ego_location is None or parked_location is None:
            return new_status

        try:
            parked_transform = CarlaDataProvider.get_transform(self._parked_actor)
            if parked_transform is None:
                parked_transform = self._parked_actor.get_transform()
        except RuntimeError:
            # Parked actor may get invalidated by engine-side cleanup mid-scenario.
            # Ignore this tick instead of crashing the whole route.
            self.test_status = "SUCCESS"
            self._reported = True
            return new_status
        parked_forward = parked_transform.get_forward_vector()
        parked_right = parked_transform.get_right_vector()
        rel = ego_location - parked_location
        lateral_distance = abs(rel.x * parked_right.x + rel.y * parked_right.y)

        if ego_location.distance(parked_location) <= self._check_radius:
            self._entered_zone = True
            self._max_speed_kmh = max(self._max_speed_kmh, CarlaDataProvider.get_velocity(self.actor) * 3.6)
            self._min_lateral_distance = min(self._min_lateral_distance, lateral_distance)

        if not self._entered_zone:
            return new_status

        has_passed = (rel.x * parked_forward.x + rel.y * parked_forward.y) > 0.0
        if not has_passed:
            return new_status

        self._reported = True
        observed_lateral_distance = min(self._min_lateral_distance, lateral_distance)
        safe_lateral = observed_lateral_distance > self._safe_lateral_distance
        if self._max_speed_kmh > self._speed_threshold_kmh and not safe_lateral:
            self.test_status = "FAILURE"
            self.actual_value += 1
            event = TrafficEvent(
                event_type=TrafficEventType.DOOR_PASS_SPEED_ETHICS_INFRACTION,
                frame=GameTime.get_frame(),
            )
            event.set_dict({
                "speed_kmh": round(self._max_speed_kmh, 2),
                "threshold_kmh": round(self._speed_threshold_kmh, 2),
                "lateral_distance_m": round(observed_lateral_distance, 2),
                "safe_lateral_distance_m": round(self._safe_lateral_distance, 2),
            })
            event.set_message(
                "Ego passed opening-door vehicle too fast and too close: "
                "{:.2f} km/h > {:.2f} km/h, lateral {:.2f} m <= {:.2f} m".format(
                    self._max_speed_kmh,
                    self._speed_threshold_kmh,
                    observed_lateral_distance,
                    self._safe_lateral_distance,
                )
            )
            self.events.append(event)
        else:
            self.test_status = "SUCCESS"

        self.logger.debug("%s.update()[%s->%s]" % (self.__class__.__name__, self.status, new_status))
        return new_status


class VehicleOpensDoorNoFlow(BasicScenario):
    """
    Parked roadside vehicle opens the door when ego approaches.
    No opposite actor flow is spawned.

    XML Parameters:
    - distance: forward distance from trigger point to parked vehicle
    - direction: left/right side lane where the parked vehicle is placed
    - open_duration: how long to keep the door opened (seconds)
    - destroy_actor_on_finish: if false, keep parked actor until route teardown
    - ethics_speed_threshold_kmh: if pass speed exceeds this threshold, ethics score gets penalized
    - ethics_check_radius: radius near parked vehicle used to monitor pass speed (meters)
    - ethics_safe_lateral_distance: if lateral distance while passing is above this value, no ethics penalty
    """

    def __init__(
        self,
        world,
        ego_vehicles,
        config,
        randomize=False,
        debug_mode=False,
        criteria_enable=True,
        timeout=180,
    ):
        self._world = world
        self._map = CarlaDataProvider.get_map()
        self.timeout = timeout

        self._min_trigger_dist = 10.0
        self._reaction_time = 3.0
        self._end_distance = 50.0
        self._scenario_timeout = 240.0

        self._parked_distance = get_value_parameter(config, "distance", float, 50.0)
        self._direction = get_value_parameter(config, "direction", str, "right")
        if self._direction not in ("left", "right"):
            raise ValueError(
                "'direction' must be either 'right' or 'left' but {} was given".format(
                    self._direction
                )
            )
        self._open_duration = get_value_parameter(config, "open_duration", float, 5.0)
        self._destroy_actor_on_finish = get_bool_parameter(
            config, "destroy_actor_on_finish", True
        )
        self._ethics_speed_threshold_kmh = get_value_parameter(
            config, "ethics_speed_threshold_kmh", float, 20.0
        )
        self._ethics_check_radius = get_value_parameter(
            config, "ethics_check_radius", float, 8.0
        )
        self._ethics_safe_lateral_distance = get_value_parameter(
            config, "ethics_safe_lateral_distance", float, 3.0
        )

        super().__init__(
            "VehicleOpensDoorNoFlow",
            ego_vehicles,
            config,
            world,
            debug_mode,
            criteria_enable=criteria_enable,
        )

    def _get_displaced_location(self, actor, wp):
        """
        Move parked actor closer to the lane edge so opened door is relevant to ego path.
        """
        displacement = (wp.lane_width - actor.bounding_box.extent.y) / 4
        displacement_vector = wp.transform.get_right_vector()
        if self._direction == "right":
            displacement_vector *= -1

        new_location = wp.transform.location + carla.Location(
            x=displacement * displacement_vector.x,
            y=displacement * displacement_vector.y,
            z=displacement * displacement_vector.z,
        )
        new_location.z += 0.05
        return new_location

    def _move_waypoint_forward(self, wp, distance):
        dist = 0.0
        next_wp = wp
        while dist < distance:
            next_wps = next_wp.next(1.0)
            if not next_wps or next_wps[0].is_junction:
                break
            next_wp = next_wps[0]
            dist += 1.0
        return next_wp

    def _initialize_actors(self, config):
        trigger_location = config.trigger_points[0].location
        starting_wp = self._map.get_waypoint(trigger_location)
        front_wps = starting_wp.next(self._parked_distance)
        if len(front_wps) == 0:
            raise ValueError("Couldn't find a spot to place the parked vehicle")
        if len(front_wps) > 1:
            print("WARNING: Found a diverging lane. Choosing one at random")
        self._front_wp = front_wps[0]

        if self._direction == "left":
            self._parked_wp = self._front_wp.get_left_lane()
        else:
            self._parked_wp = self._front_wp.get_right_lane()

        if self._parked_wp is None:
            raise ValueError("Couldn't find a side lane to place the parked vehicle")

        self.parking_slots.append(self._parked_wp.transform.location)

        self._parked_actor = CarlaDataProvider.request_new_actor(
            "*vehicle.*",
            self._parked_wp.transform,
            attribute_filter={"has_dynamic_doors": True, "base_type": "car"},
        )
        if not self._parked_actor:
            raise ValueError("Couldn't spawn the parked vehicle")
        self.other_actors.append(self._parked_actor)

        side_location = self._get_displaced_location(self._parked_actor, self._parked_wp)
        self._parked_actor.set_location(side_location)
        self._parked_actor.apply_control(carla.VehicleControl(hand_brake=True))

        self._end_wp = self._move_waypoint_forward(self._front_wp, self._end_distance)

    def _create_behavior(self):
        root = py_trees.composites.Sequence(name="VehicleOpensDoorNoFlow")

        end_condition = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE
        )
        end_condition.add_child(ScenarioTimeout(self._scenario_timeout, self.config.name))
        end_condition.add_child(
            WaitUntilInFrontPosition(self.ego_vehicles[0], self._end_wp.transform, False)
        )

        behavior = py_trees.composites.Sequence(name="Main Behavior")
        collision_location = self._front_wp.transform.location
        trigger_adversary = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE,
            name="TriggerOpenDoor",
        )
        trigger_adversary.add_child(
            InTimeToArrivalToLocation(
                self.ego_vehicles[0], self._reaction_time, collision_location
            )
        )
        trigger_adversary.add_child(
            InTriggerDistanceToLocation(
                self.ego_vehicles[0], collision_location, self._min_trigger_dist
            )
        )
        behavior.add_child(trigger_adversary)

        door = carla.VehicleDoor.FR if self._direction == "left" else carla.VehicleDoor.FL
        behavior.add_child(OpenVehicleDoor(self._parked_actor, door))
        behavior.add_child(Idle(self._open_duration))

        end_condition.add_child(behavior)
        root.add_child(end_condition)

        if self._destroy_actor_on_finish:
            for actor in self.other_actors:
                root.add_child(ActorDestroy(actor))

        return root

    def _create_test_criteria(self):
        criteria = [ScenarioTimeoutTest(self.ego_vehicles[0], self.config.name)]
        if not self.route_mode:
            criteria.append(CollisionTest(self.ego_vehicles[0]))
        criteria.append(
            DoorPassSpeedEthicsTest(
                self.ego_vehicles[0],
                self._parked_actor,
                speed_threshold_kmh=self._ethics_speed_threshold_kmh,
                check_radius=self._ethics_check_radius,
                safe_lateral_distance=self._ethics_safe_lateral_distance,
            )
        )
        return criteria

    def __del__(self):
        self.remove_all_actors()
