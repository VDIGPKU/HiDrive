#!/usr/bin/env python

"""
Lead vehicle brake scenario:

A single vehicle is spawned ahead of the ego on the same lane, drives normally with
autopilot from the start. When ego reaches the trigger point, the lead vehicle brakes,
stops for a short delay, and then resumes driving.
"""

import py_trees
import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (
    ActorDestroy, WaypointFollower, Idle, HandBrakeVehicle)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenarios.basic_scenario import BasicScenario


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
        print(f"[LeadVehicleBrake] WARN: failed to {action} autopilot: {exc}")
        return False


class LeadVehicleBrake(BasicScenario):
    """
    A lead vehicle drives normally from scenario start via autopilot.
    When ego reaches the trigger point, the lead vehicle brakes to a stop,
    waits 7 seconds by default, and then resumes driving.
    """

    def __init__(self, world, ego_vehicles, config, randomize=False,
                 debug_mode=False, criteria_enable=True, timeout=180):
        self._world = world
        self._map = CarlaDataProvider.get_map()
        self.timeout = timeout

        self._forward_distance = float(
            config.other_parameters.get('forward_distance', {}).get('value', 20))
        # Keep a fixed, practical default pause before resuming.
        # Can be overridden by XML param: resume_after.
        self._resume_after = float(
            config.other_parameters.get('resume_after', {}).get('value', 7.0))
        # Keep scenario alive briefly after resume, so motion is visible.
        self._post_resume_observe = float(
            config.other_parameters.get('post_resume_observe', {}).get('value', 3.0))
        self._brake_mode = config.other_parameters.get('brake_mode', {}).get('value', 'straight')

        super().__init__("LeadVehicleBrake",
                         ego_vehicles, config, world,
                         debug_mode, criteria_enable=criteria_enable)

    def _initialize_actors(self, config):
        ego_wp = self._map.get_waypoint(self.ego_vehicles[0].get_location())
        front_wps = ego_wp.next(self._forward_distance)
        if not front_wps:
            raise RuntimeError("Cannot find waypoint ahead of ego")
        self._lead_transform = front_wps[0].transform
        self._lead_wp = front_wps[0]

        self._lead_vehicle = CarlaDataProvider.request_new_actor(
            'vehicle.*', self._lead_transform, rolename='scenario',
            attribute_filter={'base_type': 'car', 'has_lights': True}
        )
        self.other_actors.append(self._lead_vehicle)

        # Place and start driving immediately with autopilot
        self._lead_vehicle.set_transform(self._lead_transform)
        self._lead_vehicle.set_simulate_physics(True)
        _safe_set_autopilot(self._lead_vehicle, True)

    def _create_behavior(self):
        """
        When ego reaches trigger point, this behavior executes:
        disable autopilot and brake the lead vehicle.
        brake_mode='straight': brake in place along current heading
        brake_mode='turn': follow a short turning path first, then brake and hold
        """
        behavior = py_trees.composites.Sequence("LeadVehicleBrake")

        if self._brake_mode == 'turn':
            behavior.add_child(DisableAutopilotOnly(self._lead_vehicle))
            behavior.add_child(WaypointFollower(
                self._lead_vehicle,
                4.0,
                plan=self._build_turn_brake_plan(),
                name="LeadTurnThenBrake"))
        else:
            behavior.add_child(DisableAutopilotAndBrake(self._lead_vehicle))

        behavior.add_child(HoldBrakeUntilStopped(self._lead_vehicle))
        behavior.add_child(HandBrakeVehicle(self._lead_vehicle, True))
        # Keep stopped for a short delay, then continue driving.
        behavior.add_child(Idle(self._resume_after))
        behavior.add_child(ReleaseBrakeAndResumeAutopilot(self._lead_vehicle))
        behavior.add_child(Idle(self._post_resume_observe))
        return behavior

    def _build_turn_brake_plan(self):
        """
        Build a short path that lets the lead vehicle enter the turn and then brake.
        Once a noticeable heading change is detected, only allow about 1 m more travel.
        """
        def _angle_diff_deg(a, b):
            return abs((a - b + 180.0) % 360.0 - 180.0)

        brake_plan = []
        wp = self._lead_wp
        start_yaw = wp.transform.rotation.yaw
        traveled = 0.0
        distance_after_turn = 0.0
        turn_detected = False

        for _ in range(20):
            next_wps = wp.next(1.0)
            if not next_wps:
                break

            wp = next_wps[0]
            traveled += 1.0
            brake_plan.append((wp, 1))

            yaw_delta = _angle_diff_deg(wp.transform.rotation.yaw, start_yaw)
            if not turn_detected and traveled >= 6.0 and yaw_delta >= 15.0:
                turn_detected = True
                distance_after_turn = 0.0
                continue

            if turn_detected:
                distance_after_turn += 1.0
                if distance_after_turn >= 1.0:
                    break

            if traveled >= 12.0 and yaw_delta < 10.0:
                break

        if not brake_plan:
            brake_plan.append((self._lead_wp, 1))

        return brake_plan

    def _create_test_criteria(self):
        if self.route_mode:
            return []
        return [CollisionTest(self.ego_vehicles[0])]

    def __del__(self):
        self.remove_all_actors()


class DisableAutopilotAndBrake(py_trees.behaviour.Behaviour):
    """Disable autopilot and apply full brake on the actor."""

    def __init__(self, actor, name="DisableAutopilotAndBrake"):
        super().__init__(name)
        self._actor = actor

    def update(self):
        _safe_set_autopilot(self._actor, False)
        control = carla.VehicleControl()
        control.brake = 1.0
        control.throttle = 0.0
        control.steer = 0.0
        self._actor.apply_control(control)
        # Turn on brake lights
        lights = self._actor.get_light_state()
        lights |= carla.VehicleLightState.Brake
        self._actor.set_light_state(carla.VehicleLightState(lights))
        return py_trees.common.Status.SUCCESS


class DisableAutopilotOnly(py_trees.behaviour.Behaviour):
    """Disable autopilot but keep the current motion until the next behavior takes over."""

    def __init__(self, actor, name="DisableAutopilotOnly"):
        super().__init__(name)
        self._actor = actor

    def update(self):
        _safe_set_autopilot(self._actor, False)
        return py_trees.common.Status.SUCCESS


class HoldBrakeUntilStopped(py_trees.behaviour.Behaviour):
    """Keep applying full brake until the vehicle speed drops to near zero."""

    def __init__(self, actor, speed_threshold=0.5, name="HoldBrakeUntilStopped"):
        super().__init__(name)
        self._actor = actor
        self._speed_threshold = speed_threshold

    def update(self):
        control = carla.VehicleControl()
        control.brake = 1.0
        control.throttle = 0.0
        control.steer = 0.0
        self._actor.apply_control(control)

        velocity = self._actor.get_velocity()
        speed = (velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2) ** 0.5
        if speed < self._speed_threshold:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING


class ReleaseBrakeAndResumeAutopilot(py_trees.behaviour.Behaviour):
    """Release brake/handbrake and let the actor continue with autopilot."""

    def __init__(self, actor, name="ReleaseBrakeAndResumeAutopilot"):
        super().__init__(name)
        self._actor = actor

    def update(self):
        control = carla.VehicleControl()
        control.brake = 0.0
        control.throttle = 0.0
        control.steer = 0.0
        control.hand_brake = False
        self._actor.apply_control(control)
        _safe_set_autopilot(self._actor, True)

        # Clear brake light if enabled by previous braking step.
        lights = self._actor.get_light_state()
        lights &= ~carla.VehicleLightState.Brake
        self._actor.set_light_state(carla.VehicleLightState(lights))
        return py_trees.common.Status.SUCCESS
