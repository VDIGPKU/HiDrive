#!/usr/bin/env python

# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Police interception scenario:
- Police vehicle starts behind ego
- Overtakes ego and stops ahead
- If ego bypasses police (> threshold) or collides with police, emit legal violation event
"""

from __future__ import print_function

import math

import carla
import py_trees

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (
    ActorTransformSetter,
    HandBrakeVehicle,
    Idle,
    WaypointFollower,
)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest, Criterion
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import DriveDistance, WaitUntilInFront
from srunner.scenariomanager.timer import GameTime
from srunner.scenariomanager.traffic_events import TrafficEvent, TrafficEventType
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.scenarios.highway_cut_in import ActorAlignedWithEgo, EgoRelativeSpeedWaypointFollower
from srunner.tools.scenario_helper import generate_target_waypoint_list_multilane

POLICE_FORCE_ROUTE_COMPLETION_BB_KEY = "PoliceInterceptStop_force_route_completion_100"


def _get_param(config, name, cast, default):
    if hasattr(config, "other_parameters") and name in config.other_parameters:
        try:
            return cast(config.other_parameters[name].get("value", default))
        except (TypeError, ValueError):
            return default
    return default


class DecelerateToStop(py_trees.behaviour.Behaviour):
    """
    Speed-closed-loop deceleration until full stop.
    This avoids relying on fixed brake ratios that may behave inconsistently.
    """

    def __init__(
        self,
        actor,
        decel_kmhps=8.0,
        stop_threshold_kmh=0.3,
        name="DecelerateToStop",
    ):
        super(DecelerateToStop, self).__init__(name)
        self._actor = actor
        self._decel_kmhps = max(0.5, float(decel_kmhps))
        self._stop_threshold_kmh = max(0.05, float(stop_threshold_kmh))
        self._last_time = None

    def initialise(self):
        self._last_time = GameTime.get_time()
        super(DecelerateToStop, self).initialise()

    def update(self):
        if self._actor is None or not self._actor.is_alive:
            return py_trees.common.Status.FAILURE

        now = GameTime.get_time()
        dt = 0.05 if self._last_time is None else max(0.01, now - self._last_time)
        self._last_time = now

        current_speed_mps = CarlaDataProvider.get_velocity(self._actor) or 0.0
        current_speed_kmh = current_speed_mps * 3.6
        target_speed_kmh = max(0.0, current_speed_kmh - self._decel_kmhps * dt)

        transform = CarlaDataProvider.get_transform(self._actor)
        if transform is None:
            return py_trees.common.Status.RUNNING

        fwd = transform.get_forward_vector()
        target_speed_mps = target_speed_kmh / 3.6
        self._actor.set_target_velocity(
            carla.Vector3D(fwd.x * target_speed_mps, fwd.y * target_speed_mps, 0.0)
        )

        # Keep controls neutral while target velocity ramps down.
        control = self._actor.get_control()
        control.throttle = 0.0
        control.brake = 0.0
        control.hand_brake = False
        self._actor.apply_control(control)

        if target_speed_kmh <= self._stop_threshold_kmh or current_speed_kmh <= self._stop_threshold_kmh:
            self._actor.set_target_velocity(carla.Vector3D(0.0, 0.0, 0.0))
            control = self._actor.get_control()
            control.throttle = 0.0
            control.brake = 1.0
            control.hand_brake = False
            self._actor.apply_control(control)
            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.RUNNING


class ForceActorGreenTrafficLight(py_trees.behaviour.Behaviour):
    """Keep the actor's current traffic light green while this behavior is active."""

    def __init__(self, actor, name="ForceActorGreenTrafficLight"):
        super(ForceActorGreenTrafficLight, self).__init__(name)
        self._actor = actor

    def update(self):
        if self._actor is None or not self._actor.is_alive:
            return py_trees.common.Status.RUNNING

        try:
            traffic_light = self._actor.get_traffic_light() if self._actor.is_at_traffic_light() else None
            if traffic_light is not None:
                traffic_light.set_state(carla.TrafficLightState.Green)
                traffic_light.set_green_time(10000.0)
        except RuntimeError:
            pass

        return py_trees.common.Status.RUNNING


class PoliceInterceptViolationTest(Criterion):
    """
    Emits POLICE_STOP_VIOLATION if:
    - Ego collides with police actor, or
    - After police has stopped, ego passes ahead by > bypass_distance_m
    """

    def __init__(
        self,
        ego_actor,
        police_actor,
        bypass_distance_m=5.0,
        stop_speed_threshold_kmh=0.8,
        stop_confirm_time_s=1.0,
        name="PoliceInterceptViolationTest",
    ):
        super().__init__(name, ego_actor, optional=True)
        self._police_actor = police_actor
        self._bypass_distance_m = max(0.5, float(bypass_distance_m))
        self._stop_speed_threshold_kmh = max(0.1, float(stop_speed_threshold_kmh))
        self._stop_confirm_time_s = max(0.1, float(stop_confirm_time_s))

        self._collision_sensor = None
        self._reported = False
        self._police_stopped = False
        self._stop_start_time = None

    def initialise(self):
        world = CarlaDataProvider.get_world()
        blueprint = world.get_blueprint_library().find("sensor.other.collision")
        self._collision_sensor = world.spawn_actor(blueprint, carla.Transform(), attach_to=self.actor)
        self._collision_sensor.listen(self._on_collision)
        super().initialise()

    def terminate(self, new_status):
        if self._collision_sensor is not None:
            self._collision_sensor.stop()
            self._collision_sensor.destroy()
        self._collision_sensor = None
        super().terminate(new_status)

    def _emit_violation(self, reason, message, extra=None):
        if self._reported:
            return
        self._reported = True
        self.test_status = "FAILURE"
        self.actual_value += 1

        event = TrafficEvent(TrafficEventType.POLICE_STOP_VIOLATION, GameTime.get_frame())
        data = {"reason": reason}
        if extra:
            data.update(extra)
        event.set_dict(data)
        event.set_message(message)
        self.events.append(event)

    def _on_collision(self, event):
        if self._reported or self._police_actor is None:
            return
        other = getattr(event, "other_actor", None)
        if other is None:
            return
        if other.id != self._police_actor.id:
            return

        ego_loc = CarlaDataProvider.get_location(self.actor)
        self._emit_violation(
            "collision_with_police",
            "Police intercept violation: ego collided with police vehicle",
            {
                "ego_x": round(ego_loc.x, 3) if ego_loc else None,
                "ego_y": round(ego_loc.y, 3) if ego_loc else None,
                "police_actor_id": int(self._police_actor.id),
            },
        )

    def _update_police_stop_state(self):
        if self._police_stopped or self._police_actor is None:
            return

        police_speed_kmh = CarlaDataProvider.get_velocity(self._police_actor) * 3.6
        now = GameTime.get_time()
        if police_speed_kmh <= self._stop_speed_threshold_kmh:
            if self._stop_start_time is None:
                self._stop_start_time = now
            elif (now - self._stop_start_time) >= self._stop_confirm_time_s:
                self._police_stopped = True
        else:
            self._stop_start_time = None

    def update(self):
        new_status = py_trees.common.Status.RUNNING

        if self._reported or self.actor is None or self._police_actor is None:
            return new_status

        ego_location = CarlaDataProvider.get_location(self.actor)
        police_location = CarlaDataProvider.get_location(self._police_actor)
        if ego_location is None or police_location is None:
            return new_status

        self._update_police_stop_state()
        if not self._police_stopped:
            return new_status

        police_transform = self._police_actor.get_transform()
        police_forward = police_transform.get_forward_vector()
        police_right = police_transform.get_right_vector()

        rel = ego_location - police_location
        longitudinal = rel.x * police_forward.x + rel.y * police_forward.y
        lateral = abs(rel.x * police_right.x + rel.y * police_right.y)
        distance_2d = math.hypot(rel.x, rel.y)

        if longitudinal > self._bypass_distance_m:
            self._emit_violation(
                "bypass_police_stop",
                (
                    "Police intercept violation: ego bypassed police stop (ahead {:.2f} m > {:.2f} m)"
                    .format(longitudinal, self._bypass_distance_m)
                ),
                {
                    "ahead_distance_m": round(longitudinal, 3),
                    "bypass_threshold_m": round(self._bypass_distance_m, 3),
                    "lateral_distance_m": round(lateral, 3),
                    "distance_2d_m": round(distance_2d, 3),
                    "police_actor_id": int(self._police_actor.id),
                },
            )

        return new_status


class PoliceCompliantStopCompletionTest(Criterion):
    """
    If police has fully stopped and ego stays stopped within the rear 5m window
    for a hold duration, emit ROUTE_COMPLETION=100 and request route termination.
    """

    def __init__(
        self,
        ego_actor,
        police_actor,
        behind_distance_m=5.0,
        hold_time_s=3.0,
        police_stop_speed_threshold_kmh=3.0,
        police_stop_confirm_time_s=0.4,
        ego_stop_speed_threshold_kmh=1.5,
        lateral_tolerance_m=3.0,
        force_completion_bb_key=POLICE_FORCE_ROUTE_COMPLETION_BB_KEY,
        name="PoliceCompliantStopCompletionTest",
    ):
        super().__init__(name, ego_actor, optional=True)
        self._police_actor = police_actor
        self._behind_distance_m = max(0.5, float(behind_distance_m))
        self._hold_time_s = max(0.1, float(hold_time_s))
        self._police_stop_speed_threshold_kmh = max(0.1, float(police_stop_speed_threshold_kmh))
        self._police_stop_confirm_time_s = max(0.1, float(police_stop_confirm_time_s))
        self._ego_stop_speed_threshold_kmh = max(0.1, float(ego_stop_speed_threshold_kmh))
        self._lateral_tolerance_m = max(0.5, float(lateral_tolerance_m))
        self._force_completion_bb_key = force_completion_bb_key

        self._police_stopped = False
        self._police_stop_start_time = None
        self._ego_hold_start_time = None
        self._completion_reported = False

    def _update_police_stop_state(self):
        if self._police_stopped or self._police_actor is None:
            return

        police_speed_kmh = (CarlaDataProvider.get_velocity(self._police_actor) or 0.0) * 3.6
        now = GameTime.get_time()
        if police_speed_kmh <= self._police_stop_speed_threshold_kmh:
            if self._police_stop_start_time is None:
                self._police_stop_start_time = now
            elif (now - self._police_stop_start_time) >= self._police_stop_confirm_time_s:
                self._police_stopped = True
        else:
            self._police_stop_start_time = None

    def _emit_forced_completion(self):
        if self._completion_reported:
            return

        self._completion_reported = True
        self.test_status = "SUCCESS"
        self.actual_value = 100.0

        event = TrafficEvent(event_type=TrafficEventType.ROUTE_COMPLETION, frame=GameTime.get_frame())
        event.set_dict(
            {
                "route_completed": 100.0,
                "reason": "police_compliant_stop",
                "behind_distance_m": self._behind_distance_m,
                "hold_time_s": self._hold_time_s,
            }
        )
        event.set_message(
            "Police compliant stop reached: ego stopped behind police; route marked complete at 100%."
        )
        self.events.append(event)

        blackboard = py_trees.blackboard.Blackboard()
        blackboard.set(self._force_completion_bb_key, True, overwrite=True)

    def update(self):
        new_status = py_trees.common.Status.RUNNING

        if self._completion_reported or self.actor is None or self._police_actor is None:
            return new_status

        self._update_police_stop_state()
        if not self._police_stopped:
            self._ego_hold_start_time = None
            return new_status

        ego_location = CarlaDataProvider.get_location(self.actor)
        police_location = CarlaDataProvider.get_location(self._police_actor)
        police_transform = CarlaDataProvider.get_transform(self._police_actor)
        ego_transform = CarlaDataProvider.get_transform(self.actor)
        if ego_location is None or police_location is None or police_transform is None:
            return new_status

        rel = ego_location - police_location
        police_forward = police_transform.get_forward_vector()
        police_right = police_transform.get_right_vector()

        longitudinal = rel.x * police_forward.x + rel.y * police_forward.y
        lateral = abs(rel.x * police_right.x + rel.y * police_right.y)
        distance_2d = math.hypot(rel.x, rel.y)

        police_half_len = max(0.5, float(getattr(self._police_actor.bounding_box.extent, "x", 1.0)))
        ego_half_len = max(0.5, float(getattr(self.actor.bounding_box.extent, "x", 1.0)))
        # Positive when ego is behind police by bumper-to-bumper distance.
        rear_gap_m = max(0.0, (-longitudinal) - (police_half_len + ego_half_len))

        ego_speed_kmh = (CarlaDataProvider.get_velocity(self.actor) or 0.0) * 3.6
        behind_by_police = longitudinal <= 1.0
        behind_by_ego = False
        if ego_transform is not None:
            ego_forward = ego_transform.get_forward_vector()
            rel_police_from_ego = police_location - ego_location
            ego_longitudinal = (
                rel_police_from_ego.x * ego_forward.x + rel_police_from_ego.y * ego_forward.y
            )
            behind_by_ego = ego_longitudinal >= -1.0

        # Robust rear-stop detection:
        # - rear bumper gap <= configured behind distance
        # - judged as "behind-ish" by police frame or ego frame
        # - optional lateral cap kept loose
        in_rear_zone = (
            rear_gap_m <= self._behind_distance_m
            and (behind_by_police or behind_by_ego)
            and lateral <= max(self._lateral_tolerance_m, self._behind_distance_m + 1.5)
            and distance_2d <= (self._behind_distance_m + police_half_len + ego_half_len + 2.0)
        )
        ego_stopped = ego_speed_kmh <= self._ego_stop_speed_threshold_kmh

        now = GameTime.get_time()
        if in_rear_zone and ego_stopped:
            if self._ego_hold_start_time is None:
                self._ego_hold_start_time = now
            elif (now - self._ego_hold_start_time) >= self._hold_time_s:
                self._emit_forced_completion()
        else:
            self._ego_hold_start_time = None

        return new_status


class PoliceInterceptStop(BasicScenario):
    """
    Police starts behind ego, overtakes, then brakes to a stop in front.
    """

    _DEFAULT_POLICE_BLUEPRINT_PATTERNS = [
        "vehicle.dodge.charger_police_2020",
        "vehicle.dodge.charger_police",
        "vehicle.*police*",
        "vehicle.*cop*",
    ]

    _SPAWN_DISTANCE_OFFSETS = [0, 5, -5, 10, -10, 15, -15, 20, -20]

    def __init__(self, world, ego_vehicles, config, debug_mode=False, criteria_enable=True, timeout=180):
        self._world = world
        self._map = CarlaDataProvider.get_map()
        self.timeout = timeout

        self._trigger_location = config.trigger_points[0].location
        self._reference_waypoint = self._map.get_waypoint(self._trigger_location)

        self._spawn_distance_m = _get_param(config, "spawn_distance", float, 16.0)
        self._speed_increment_kmh = _get_param(config, "speed_increment", float, 18.0)
        self._lead_distance_m = _get_param(config, "lead_distance", float, 8.0)
        self._brake_value = _get_param(config, "brake_value", float, 0.5)
        self._stop_decel_kmhps = _get_param(
            config, "stop_deceleration", float, max(3.0, self._brake_value * 12.0)
        )
        self._stop_hold_time_s = _get_param(config, "stop_hold_time", float, 60.0)
        self._compliant_stop_distance_m = _get_param(config, "compliant_stop_distance", float, 5.0)
        self._compliant_stop_hold_time_s = _get_param(config, "compliant_stop_hold_time", float, 3.0)
        self._compliant_stop_lateral_tolerance_m = _get_param(
            config, "compliant_stop_lateral_tolerance", float, 3.0
        )
        self._bypass_distance_m = _get_param(config, "bypass_distance", float, 5.0)
        self._police_vehicle_model = str(_get_param(config, "police_vehicle_model", str, "")).strip()
        self._cut_in_completion_distance = _get_param(config, "cut_in_completion_distance", float, 8.0)
        self._overtake_side = str(_get_param(config, "overtake_side", str, "auto")).strip().lower()
        self._cut_in_direction_override = str(_get_param(config, "cut_in_direction", str, "")).strip().lower()

        self._police_start_transform = None
        self._police_start_waypoint = None
        self._cut_in_direction = None

        super().__init__(
            "PoliceInterceptStop",
            ego_vehicles,
            config,
            world,
            debug_mode,
            criteria_enable=criteria_enable,
        )

    def _resolve_police_blueprint_id(self):
        bp_library = self._world.get_blueprint_library()
        patterns = []
        if self._police_vehicle_model:
            patterns.append(self._police_vehicle_model)
        patterns.extend(self._DEFAULT_POLICE_BLUEPRINT_PATTERNS)

        for pattern in patterns:
            candidates = bp_library.filter(pattern)
            if candidates:
                return candidates[0].id

        # Last fallback: any drivable vehicle to avoid setup hard-fail.
        fallback = bp_library.filter("vehicle.*")
        if fallback:
            return fallback[0].id
        return None

    @staticmethod
    def _is_same_direction_lane(base_wp, side_wp):
        if base_wp is None or side_wp is None:
            return False
        if side_wp.lane_type != carla.LaneType.Driving:
            return False
        if side_wp.road_id != base_wp.road_id:
            return False
        if base_wp.lane_id == 0 or side_wp.lane_id == 0:
            return True
        return base_wp.lane_id * side_wp.lane_id > 0

    def _pick_overtake_lane(self, base_wp):
        left_wp = base_wp.get_left_lane()
        right_wp = base_wp.get_right_lane()
        left_ok = self._is_same_direction_lane(base_wp, left_wp)
        right_ok = self._is_same_direction_lane(base_wp, right_wp)

        def select_side(side):
            if side == "left" and left_ok:
                return left_wp, "right"
            if side == "right" and right_ok:
                return right_wp, "left"
            return None, None

        if self._cut_in_direction_override in ("left", "right"):
            preferred_side = "right" if self._cut_in_direction_override == "left" else "left"
            spawn_wp, cut_in_dir = select_side(preferred_side)
            if spawn_wp is not None:
                return spawn_wp, cut_in_dir

        if self._overtake_side in ("left", "right"):
            spawn_wp, cut_in_dir = select_side(self._overtake_side)
            if spawn_wp is not None:
                return spawn_wp, cut_in_dir

        spawn_wp, cut_in_dir = select_side("left")
        if spawn_wp is not None:
            return spawn_wp, cut_in_dir

        spawn_wp, cut_in_dir = select_side("right")
        if spawn_wp is not None:
            return spawn_wp, cut_in_dir

        return base_wp, None

    def _get_start_transform_candidates(self):
        adjacent_candidates = []
        fallback_candidates = []
        seen = set()
        for offset in self._SPAWN_DISTANCE_OFFSETS:
            distance = max(8.0, self._spawn_distance_m + offset)
            prev_wps = self._reference_waypoint.previous(distance)
            if not prev_wps:
                continue

            base_wp = prev_wps[0]
            spawn_wp, cut_in_direction = self._pick_overtake_lane(base_wp)
            transform = spawn_wp.transform
            key = (
                round(transform.location.x, 2),
                round(transform.location.y, 2),
                round(transform.location.z, 2),
                round(transform.rotation.yaw, 1),
            )
            if key in seen:
                continue
            seen.add(key)
            candidate = (transform, spawn_wp, cut_in_direction)
            if cut_in_direction is None:
                fallback_candidates.append(candidate)
            else:
                adjacent_candidates.append(candidate)
        return adjacent_candidates + fallback_candidates

    def _initialize_actors(self, config):
        bp_id = self._resolve_police_blueprint_id()
        if not bp_id:
            raise ValueError("PoliceInterceptStop: unable to resolve police blueprint")

        actor = None
        for transform, start_wp, cut_in_direction in self._get_start_transform_candidates():
            actor = CarlaDataProvider.request_new_actor(bp_id, transform)
            if actor is not None:
                self._police_start_transform = transform
                self._police_start_waypoint = start_wp
                self._cut_in_direction = cut_in_direction
                break

        if actor is None:
            raise ValueError("PoliceInterceptStop: unable to spawn police actor")

        actor.set_simulate_physics(False)
        hidden = actor.get_location()
        hidden.z -= 200.0
        actor.set_location(hidden)

        if hasattr(carla, "VehicleLightState"):
            try:
                actor.set_light_state(carla.VehicleLightState(
                    carla.VehicleLightState.Special1 | carla.VehicleLightState.Special2))
            except RuntimeError:
                pass

        self.other_actors.append(actor)

    def _create_behavior(self):
        sequence = py_trees.composites.Sequence(name="PoliceInterceptStop")
        police = self.other_actors[0]
        ego = self.ego_vehicles[0]

        sequence.add_child(ActorTransformSetter(police, self._police_start_transform))

        # 1) HighwayCutIn-style same-lane fast advance
        same_lane_plan = []
        wp = self._police_start_waypoint
        for _ in range(30):
            same_lane_plan.append((wp, 1))
            next_wps = wp.next(2.0)
            if not next_wps:
                break
            wp = next_wps[0]

        drive_until_close = py_trees.composites.Parallel(
            "DriveUntilClose",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE,
        )
        drive_until_close.add_child(EgoRelativeSpeedWaypointFollower(
            police,
            ego,
            speed_delta_kmh=self._speed_increment_kmh,
            plan=same_lane_plan,
            name="CutInRelativeSpeedFollow",
        ))
        drive_until_close.add_child(ActorAlignedWithEgo(police, ego))
        sequence.add_child(drive_until_close)

        # 2) HighwayCutIn-style lane-change cut-in
        estimated_wp = self._police_start_waypoint.next(30)[0] if self._police_start_waypoint.next(30) else self._police_start_waypoint
        cut_in_direction = self._cut_in_direction if self._cut_in_direction in ("left", "right") else "right"
        cut_in_plan, _ = generate_target_waypoint_list_multilane(
            estimated_wp,
            change=cut_in_direction,
            distance_same_lane=0,
            distance_other_lane=120,
            total_lane_change_distance=self._cut_in_completion_distance,
            check=False,
        )
        cut_in_until_ready = py_trees.composites.Parallel(
            "CutInUntilReadyToStop",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE,
        )
        if cut_in_plan:
            cut_in_until_ready.add_child(WaypointFollower(police, 10, plan=cut_in_plan))

        ready_to_stop = py_trees.composites.Sequence("ReadyToStopAfterCutIn")
        ready_to_stop.add_child(WaitUntilInFront(police, ego, check_distance=False))
        ready_to_stop.add_child(DriveDistance(police, self._lead_distance_m))
        cut_in_until_ready.add_child(ready_to_stop)
        sequence.add_child(cut_in_until_ready)

        # 3) Only post-overtake difference from HighwayCutIn: decelerate to a full stop
        sequence.add_child(DecelerateToStop(
            police,
            decel_kmhps=self._stop_decel_kmhps,
            stop_threshold_kmh=0.3,
        ))
        sequence.add_child(HandBrakeVehicle(police, True))
        sequence.add_child(Idle(self._stop_hold_time_s))

        with_ignore_lights = py_trees.composites.Parallel(
            name="PoliceInterceptStop_IgnoreTrafficLights",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE,
        )
        with_ignore_lights.add_child(sequence)
        with_ignore_lights.add_child(ForceActorGreenTrafficLight(police))
        return with_ignore_lights

    def _create_test_criteria(self):
        criteria = [
            PoliceInterceptViolationTest(
                self.ego_vehicles[0],
                self.other_actors[0],
                bypass_distance_m=self._bypass_distance_m,
            ),
            PoliceCompliantStopCompletionTest(
                self.ego_vehicles[0],
                self.other_actors[0],
                behind_distance_m=self._compliant_stop_distance_m,
                hold_time_s=self._compliant_stop_hold_time_s,
                lateral_tolerance_m=self._compliant_stop_lateral_tolerance_m,
                force_completion_bb_key=POLICE_FORCE_ROUTE_COMPLETION_BB_KEY,
            ),
        ]
        if not self.route_mode:
            criteria.append(CollisionTest(self.ego_vehicles[0]))
        return criteria

    def __del__(self):
        self.remove_all_actors()
