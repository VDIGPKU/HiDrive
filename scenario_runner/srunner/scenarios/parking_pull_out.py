#!/usr/bin/env python

"""
Parking pull-out scenario:

A parked vehicle on the roadside starts moving and merges into the ego's lane,
cutting in front of the ego. Configurable turn signal usage.
"""

import py_trees
import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (
    ActorDestroy, WaypointFollower, Idle)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenarios.basic_scenario import BasicScenario


def convert_dict_to_location(actor_dict):
    return carla.Location(
        x=float(actor_dict['x']),
        y=float(actor_dict['y']),
        z=float(actor_dict['z'])
    )


def _safe_set_autopilot(actor, enabled):
    """Best-effort autopilot toggle using the active TM port."""
    if actor is None:
        return False
    if hasattr(actor, "is_alive") and not actor.is_alive:
        return False

    tm_port = CarlaDataProvider.get_traffic_manager_port()
    try:
        actor.set_autopilot(enabled, tm_port)
        return True
    except (RuntimeError, TypeError):
        pass

    try:
        actor.set_autopilot(enabled)
        return True
    except RuntimeError as exc:
        action = "enable" if enabled else "disable"
        print(f"[ParkingPullOut] WARN: failed to {action} autopilot: {exc}")
        return False


class ParkingPullOut(BasicScenario):
    """
    A parked vehicle on the roadside pulls out and merges into the ego's lane.
    """

    def __init__(self, world, ego_vehicles, config, randomize=False,
                 debug_mode=False, criteria_enable=True, timeout=180):
        self._world = world
        self._map = CarlaDataProvider.get_map()
        self.timeout = timeout

        self._start_location = convert_dict_to_location(
            config.other_parameters['parked_location'])
        self._use_signal = config.other_parameters.get(
            'use_signal', {}).get('value', 'true') == 'true'
        self._merge_speed = float(
            config.other_parameters.get('merge_speed', {}).get('value', 15))
        self._after_merge_distance = float(
            config.other_parameters.get('after_merge_distance', {}).get('value', 80))
        self._lane_change_distance = float(
            config.other_parameters.get('lane_change_distance', {}).get('value', 8))

        super().__init__("ParkingPullOut",
                         ego_vehicles, config, world,
                         debug_mode, criteria_enable=criteria_enable)

    def _initialize_actors(self, config):
        self._parked_wp = self._map.get_waypoint(
            self._start_location,
            lane_type=carla.LaneType.Any)
        self._parked_transform = self._parked_wp.transform

        # Find the driving lane to the left for merge target
        self._driving_wp = self._map.get_waypoint(self._start_location)

        self._parked_vehicle = CarlaDataProvider.request_new_actor(
            'vehicle.*', self._parked_transform, rolename='scenario',
            attribute_filter={'base_type': 'car', 'has_lights': True}
        )
        self.other_actors.append(self._parked_vehicle)

        # Place vehicle at parking position, visible from start, with handbrake
        self._parked_vehicle.set_transform(self._parked_transform)
        self._parked_vehicle.set_simulate_physics(True)
        self._parked_vehicle.apply_control(carla.VehicleControl(hand_brake=True))

    def _create_behavior(self):
        behavior = py_trees.composites.Sequence("ParkingPullOut")

        # Turn on signal if configured
        if self._use_signal:
            behavior.add_child(SetTurnSignal(self._parked_vehicle, 'left'))

        # Release handbrake
        behavior.add_child(ReleaseHandbrake(self._parked_vehicle))

        # Build merge path manually:
        # 1. Short straight from parking spot
        # 2. Interpolate into driving lane
        # 3. Continue on driving lane
        plan = []

        # Phase 1: drive forward a bit in parking lane
        wp = self._parked_wp
        for _ in range(2):
            next_wps = wp.next(2.0)
            if not next_wps:
                break
            wp = next_wps[0]
            plan.append((wp, 1))

        # Phase 2: use driving lane waypoints offset to create merge
        merge_start_wp = self._driving_wp.next(4.0)
        if merge_start_wp:
            merge_wp = merge_start_wp[0]
        else:
            merge_wp = self._driving_wp

        # Phase 2: merge into driving lane
        steps = int(self._lane_change_distance / 2.0)
        for i in range(steps):
            next_wps = merge_wp.next(2.0)
            if not next_wps:
                break
            merge_wp = next_wps[0]
            plan.append((merge_wp, 1))

        # Execute merge path
        if plan:
            behavior.add_child(WaypointFollower(
                self._parked_vehicle, self._merge_speed, plan=plan))

        # Turn off signal after merge
        if self._use_signal:
            behavior.add_child(SetTurnSignal(self._parked_vehicle, 'off'))

        # Switch to autopilot so speed matches environment traffic
        behavior.add_child(EnableAutopilot(self._parked_vehicle))

        # Keep alive until ego passes
        behavior.add_child(Idle(self._after_merge_distance))

        behavior.add_child(ActorDestroy(self._parked_vehicle))
        return behavior

    def _create_test_criteria(self):
        if self.route_mode:
            return []
        return [CollisionTest(self.ego_vehicles[0])]

    def __del__(self):
        self.remove_all_actors()


class ReleaseHandbrake(py_trees.behaviour.Behaviour):
    """Release handbrake on the actor."""

    def __init__(self, actor, name="ReleaseHandbrake"):
        super().__init__(name)
        self._actor = actor

    def update(self):
        control = carla.VehicleControl()
        control.hand_brake = False
        self._actor.apply_control(control)
        return py_trees.common.Status.SUCCESS


class EnableAutopilot(py_trees.behaviour.Behaviour):
    """Enable autopilot on the actor so it follows traffic flow speed."""

    def __init__(self, actor, name="EnableAutopilot"):
        super().__init__(name)
        self._actor = actor

    def update(self):
        _safe_set_autopilot(self._actor, True)
        return py_trees.common.Status.SUCCESS


class SetTurnSignal(py_trees.behaviour.Behaviour):
    """Set turn signal lights on the actor."""

    def __init__(self, actor, direction='left', name="SetTurnSignal"):
        super().__init__(name)
        self._actor = actor
        self._direction = direction

    def update(self):
        lights = self._actor.get_light_state()
        # Clear existing turn signals
        lights &= ~carla.VehicleLightState.LeftBlinker
        lights &= ~carla.VehicleLightState.RightBlinker
        if self._direction == 'left':
            lights |= carla.VehicleLightState.LeftBlinker
        elif self._direction == 'right':
            lights |= carla.VehicleLightState.RightBlinker
        self._actor.set_light_state(carla.VehicleLightState(lights))
        return py_trees.common.Status.SUCCESS
