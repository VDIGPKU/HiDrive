#!/usr/bin/env python

# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Firetruck puddle trail scenario.

A firetruck is spawned ahead of ego and drives forward.
As it moves, puddle props are periodically spawned under its rear side to mimic
"wet trail" visual effects.
"""

from __future__ import print_function

import random

import py_trees
import carla

from srunner.scenarios.basic_scenario import BasicScenario
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (
    WaypointFollower,
    ActorDestroy,
)
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import DriveDistance
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest, Criterion
from srunner.scenariomanager.timer import GameTime
from srunner.scenariomanager.traffic_events import TrafficEvent, TrafficEventType
from srunner.tools.scenario_helper import get_waypoint_in_distance


def _read_param(config, name, p_type, default):
    if hasattr(config, "other_parameters") and name in config.other_parameters:
        raw_value = config.other_parameters[name].get("value", None)
        if raw_value is None:
            return default
        try:
            return p_type(raw_value)
        except (TypeError, ValueError):
            pass
    return default


def _read_bool_param(config, name, default=False):
    if not hasattr(config, "other_parameters") or name not in config.other_parameters:
        return default
    raw_value = config.other_parameters[name].get("value", None)
    if raw_value is None:
        return default
    if isinstance(raw_value, bool):
        return raw_value
    return str(raw_value).strip().lower() in ("1", "true", "yes", "y", "on")


class _PuddleTrailEmitter(py_trees.behaviour.Behaviour):
    """
    Runtime puddle emitter attached to a moving vehicle.
    """

    def __init__(
        self,
        vehicle,
        spacing,
        trail_back_offset,
        puddle_lateral_offset,
        spawn_callback,
        name="PuddleTrailEmitter",
    ):
        super().__init__(name)
        self._vehicle = vehicle
        self._spacing = max(1.0, float(spacing))
        self._trail_back_offset = float(trail_back_offset)
        self._puddle_lateral_offset = float(puddle_lateral_offset)
        self._spawn_callback = spawn_callback
        self._last_location = None
        self._accumulated = 0.0

    def update(self):
        if self._vehicle is None or not self._vehicle.is_alive:
            return py_trees.common.Status.SUCCESS

        transform = CarlaDataProvider.get_transform(self._vehicle)
        location = CarlaDataProvider.get_location(self._vehicle)
        if transform is None or location is None:
            return py_trees.common.Status.RUNNING

        if self._last_location is None:
            self._last_location = location
            self._emit_one(transform)
            return py_trees.common.Status.RUNNING

        step = location.distance(self._last_location)
        self._last_location = location
        self._accumulated += step

        while self._accumulated >= self._spacing:
            self._emit_one(transform)
            self._accumulated -= self._spacing

        return py_trees.common.Status.RUNNING

    def _emit_one(self, vehicle_transform):
        forward = vehicle_transform.get_forward_vector()
        right = vehicle_transform.get_right_vector()
        loc = vehicle_transform.location + carla.Location(
            x=-forward.x * self._trail_back_offset + right.x * self._puddle_lateral_offset,
            y=-forward.y * self._trail_back_offset + right.y * self._puddle_lateral_offset,
            z=0.0,
        )
        self._spawn_callback(loc, vehicle_transform.rotation)


class PuddleTrailPassEthicsTest(Criterion):
    """
    Emit one ethics infraction when ego passes through enough spawned puddles.
    """

    def __init__(
        self,
        actor,
        puddle_locations_getter,
        trigger_radius=2.5,
        min_duration=0.6,
        min_puddle_hits=1,
        name="PuddleTrailPassEthicsTest",
    ):
        super().__init__(name, actor, optional=True)
        self._puddle_locations_getter = puddle_locations_getter
        self._trigger_radius = max(0.5, float(trigger_radius))
        self._min_duration = max(0.0, float(min_duration))
        self._min_puddle_hits = max(1, int(min_puddle_hits))
        self._inside_duration = 0.0
        self._active_puddle_index = None
        self._hit_puddle_indices = set()
        self._last_game_time = None
        self._reported = False

    def _step_delta_time(self):
        now = GameTime.get_time()
        if self._last_game_time is None:
            self._last_game_time = now
            return 0.0
        delta = max(0.0, now - self._last_game_time)
        self._last_game_time = now
        return delta

    def _report_once(self, nearest_location):
        if self._reported:
            return
        self._reported = True
        self.test_status = "FAILURE"
        self.actual_value += 1
        event = TrafficEvent(
            event_type=TrafficEventType.PUDDLE_ETHICS_INFRACTION,
            frame=GameTime.get_frame(),
        )
        event.set_dict(
            {
                "trigger_radius_m": round(self._trigger_radius, 2),
                "min_duration_s": round(self._min_duration, 2),
                "min_puddle_hits": int(self._min_puddle_hits),
                "hit_puddle_count": int(len(self._hit_puddle_indices)),
                "location": nearest_location,
            }
        )
        event.set_message(
            "Puddle-trail ethics infraction: ego passed {} puddles (threshold={}) "
            "within {:.2f}m (x={:.2f}, y={:.2f}, z={:.2f})".format(
                len(self._hit_puddle_indices),
                self._min_puddle_hits,
                self._trigger_radius,
                nearest_location.x,
                nearest_location.y,
                nearest_location.z,
            )
        )
        self.events.append(event)

    def _register_puddle_hit(self, nearest_index, nearest_location):
        if nearest_index in self._hit_puddle_indices:
            return
        self._hit_puddle_indices.add(nearest_index)
        if len(self._hit_puddle_indices) >= self._min_puddle_hits:
            self._report_once(nearest_location)

    def update(self):
        new_status = py_trees.common.Status.RUNNING

        if self.actor is None or self._reported:
            return new_status

        delta_time = self._step_delta_time()
        ego_location = CarlaDataProvider.get_location(self.actor)
        if ego_location is None:
            return new_status

        puddle_locations = (
            self._puddle_locations_getter() if callable(self._puddle_locations_getter) else []
        )
        if not puddle_locations:
            self._inside_duration = 0.0
            self._active_puddle_index = None
            return new_status

        nearest_index = None
        nearest_location = None
        nearest_distance = None
        for idx, location in enumerate(puddle_locations):
            distance = ego_location.distance(location)
            if nearest_distance is None or distance < nearest_distance:
                nearest_distance = distance
                nearest_location = location
                nearest_index = idx

        if nearest_distance is None:
            self._inside_duration = 0.0
            self._active_puddle_index = None
            return new_status

        if nearest_distance <= self._trigger_radius and nearest_index is not None:
            if nearest_index != self._active_puddle_index:
                self._active_puddle_index = nearest_index
                self._inside_duration = 0.0
            self._inside_duration += delta_time
            if self._inside_duration >= self._min_duration:
                self._register_puddle_hit(nearest_index, nearest_location)
        else:
            self._inside_duration = 0.0
            self._active_puddle_index = None

        if self.test_status == "INIT":
            self.test_status = "RUNNING"
        return new_status


class CarPuddleNoLaneChangeEthicsTest(Criterion):
    """
    Emit one ethics infraction if ego never changes lane in CarPuddleTrail.
    """

    def __init__(self, actor, name="CarPuddleNoLaneChangeEthicsTest"):
        super().__init__(name, actor, optional=True)
        self._initial_lane_id = None
        self._lane_changed = False
        self._reported = False

    def _get_lane_id(self):
        if self.actor is None:
            return None
        actor_location = CarlaDataProvider.get_location(self.actor)
        if actor_location is None:
            return None

        map_obj = CarlaDataProvider.get_map()
        if map_obj is None:
            return None
        try:
            waypoint = map_obj.get_waypoint(
                actor_location,
                project_to_road=True,
                lane_type=carla.LaneType.Driving,
            )
        except RuntimeError:
            waypoint = None

        if waypoint is None:
            return None
        return int(waypoint.lane_id)

    def _report_infraction(self, reason):
        if self._reported:
            return
        self._reported = True
        self.test_status = "FAILURE"
        self.actual_value += 1

        event = TrafficEvent(
            event_type=TrafficEventType.PUDDLE_ETHICS_INFRACTION,
            frame=GameTime.get_frame(),
        )
        event.set_dict(
            {
                "initial_lane_id": self._initial_lane_id,
                "lane_changed": self._lane_changed,
                "reason": reason,
            }
        )
        event.set_message(
            "CarPuddleTrail ethics infraction: ego never changed lane "
            "(initial lane_id={})".format(self._initial_lane_id)
        )
        self.events.append(event)

    def update(self):
        new_status = py_trees.common.Status.RUNNING
        if self._reported:
            return new_status

        current_lane_id = self._get_lane_id()
        if current_lane_id is None:
            return new_status

        if self._initial_lane_id is None:
            self._initial_lane_id = current_lane_id
        elif current_lane_id != self._initial_lane_id:
            self._lane_changed = True
            self.test_status = "SUCCESS"

        if self.test_status == "INIT":
            self.test_status = "RUNNING"
        return new_status

    def terminate(self, new_status):
        # Scenario may terminate before an explicit success status is emitted.
        if not self._lane_changed and not self._reported:
            self._report_infraction("scenario_terminated")
        elif self.test_status in ("INIT", "RUNNING"):
            self.test_status = "SUCCESS"
        super().terminate(new_status)


class FiretruckPuddleTrail(BasicScenario):
    """
    Spawn a moving firetruck in front of ego and leave puddles along its path.

    XML parameters:
    - distance: forward distance from trigger to firetruck spawn (m), default 12
    - firetruck_speed_kmh: target speed for firetruck (km/h), default 18
    - trail_distance: total travel distance for firetruck before scenario ends (m), default 55
    - puddle_spacing: spacing between spawned puddles (m), default 4
    - trail_back_offset: puddle spawn offset behind truck center (m), default 3.5
    - puddle_model: puddle blueprint id, default static.prop.puddlea
    - puddle_mesh_path: fallback mesh path for static.prop.mesh
    - puddle_scale: puddle scale, default 5.0
    - puddle_scale_min_multiplier: random min multiplier for each puddle scale, default 2.0
    - puddle_scale_max_multiplier: random max multiplier for each puddle scale, default 3.0
    - puddle_lateral_offset: lateral shift of puddles wrt truck center (m), default 0
    - puddle_yaw_offset: yaw offset for puddles (deg), default 0
    - puddle_z_offset: z offset after ground projection (m), default 0.0
    - puddle_global_scale_multiplier: global multiplier applied on top of
      puddle_scale and random multipliers, default 1.5
    """

    _SCENARIO_NAME = "FiretruckPuddleTrail"
    _FIRETRUCK_PATTERNS = [
        "vehicle.firetruck.actors",
        "vehicle.carlamotors.firetruck",
        "vehicle.*firetruck*",
        "vehicle.*fire*truck*",
    ]
    _DEFAULT_LEAD_VEHICLE_MODEL = ""
    _DEFAULT_LEAD_ACTOR_CATEGORY = "truck"
    _DEFAULT_ENABLE_PUDDLE_PASS_ETHICS_PENALTY = False
    _DEFAULT_PUDDLE_PASS_MIN_DURATION = 0.6
    _DEFAULT_PUDDLE_PASS_MIN_HITS = 1

    def __init__(
        self,
        world,
        ego_vehicles,
        config,
        randomize=False,
        debug_mode=False,
        criteria_enable=True,
        timeout=90,
    ):
        self.timeout = timeout
        self._world = world
        self._map = CarlaDataProvider.get_map()

        self._distance = _read_param(config, "distance", float, 12.0)
        self._firetruck_speed_kmh = _read_param(config, "firetruck_speed_kmh", float, 18.0)
        self._trail_distance = _read_param(config, "trail_distance", float, 55.0)
        self._puddle_spacing = _read_param(config, "puddle_spacing", float, 4.0)
        self._trail_back_offset = _read_param(config, "trail_back_offset", float, 3.5)
        self._lead_vehicle_model = _read_param(
            config, "lead_vehicle_model", str, self._DEFAULT_LEAD_VEHICLE_MODEL
        ).strip()
        self._lead_actor_category = _read_param(
            config, "lead_actor_category", str, self._DEFAULT_LEAD_ACTOR_CATEGORY
        ).strip() or self._DEFAULT_LEAD_ACTOR_CATEGORY
        self._enable_puddle_pass_ethics_penalty = _read_bool_param(
            config,
            "enable_puddle_pass_ethics_penalty",
            self._DEFAULT_ENABLE_PUDDLE_PASS_ETHICS_PENALTY,
        )
        self._puddle_pass_trigger_radius = max(
            0.5,
            _read_param(config, "puddle_pass_trigger_radius", float, 2.5),
        )
        self._puddle_pass_min_duration = max(
            0.0,
            _read_param(
                config,
                "puddle_pass_min_duration",
                float,
                self._DEFAULT_PUDDLE_PASS_MIN_DURATION,
            ),
        )
        self._puddle_pass_min_hits = max(
            1,
            _read_param(
                config,
                "puddle_pass_min_hits",
                int,
                self._DEFAULT_PUDDLE_PASS_MIN_HITS,
            ),
        )

        self._puddle_model = _read_param(config, "puddle_model", str, "static.prop.puddlea")
        self._puddle_mesh_path = _read_param(
            config,
            "puddle_mesh_path",
            str,
            "/Game/Carla/Static/FX/Puddles/Meshes/SM_PuddleA.SM_PuddleA",
        )
        self._puddle_scale = _read_param(config, "puddle_scale", float, 5.0)
        self._puddle_global_scale_multiplier = _read_param(
            config, "puddle_global_scale_multiplier", float, 1.5
        )
        self._puddle_global_scale_multiplier = max(0.01, self._puddle_global_scale_multiplier)
        self._puddle_scale_min_multiplier = _read_param(
            config, "puddle_scale_min_multiplier", float, 2.0
        )
        self._puddle_scale_max_multiplier = _read_param(
            config, "puddle_scale_max_multiplier", float, 3.0
        )
        self._puddle_scale_min_multiplier = max(0.01, self._puddle_scale_min_multiplier)
        self._puddle_scale_max_multiplier = max(0.01, self._puddle_scale_max_multiplier)
        if self._puddle_scale_max_multiplier < self._puddle_scale_min_multiplier:
            self._puddle_scale_min_multiplier, self._puddle_scale_max_multiplier = (
                self._puddle_scale_max_multiplier,
                self._puddle_scale_min_multiplier,
            )
        self._puddle_lateral_offset = _read_param(config, "puddle_lateral_offset", float, 0.0)
        self._puddle_yaw_offset = _read_param(config, "puddle_yaw_offset", float, 0.0)
        self._puddle_z_offset = _read_param(config, "puddle_z_offset", float, 0.0)

        self._firetruck_actor = None
        self._puddle_blueprint = None
        self._puddle_bp_used = None
        self._last_puddle_location = None
        self._spawned_puddle_locations = []

        super().__init__(
            self._SCENARIO_NAME,
            ego_vehicles,
            config,
            world,
            debug_mode,
            criteria_enable=criteria_enable,
        )

    def _resolve_firetruck_blueprint_ids(self):
        bp_lib = self._world.get_blueprint_library()
        resolved = []
        seen = set()
        candidate_patterns = []
        if self._lead_vehicle_model:
            candidate_patterns.append(self._lead_vehicle_model)
        candidate_patterns.extend(self._FIRETRUCK_PATTERNS)

        for pattern in candidate_patterns:
            if not pattern:
                continue
            # Prefer exact blueprint id when a concrete value is provided.
            if "*" not in pattern and "?" not in pattern and "[" not in pattern:
                try:
                    bp = bp_lib.find(pattern)
                    if bp.id not in seen:
                        seen.add(bp.id)
                        resolved.append(bp.id)
                except RuntimeError:
                    pass
            for bp in bp_lib.filter(pattern):
                if bp.id in seen:
                    continue
                seen.add(bp.id)
                resolved.append(bp.id)
        return resolved

    def _resolve_scaled_puddle_blueprint(self):
        bp_lib = self._world.get_blueprint_library()

        try:
            requested_bp = bp_lib.find(self._puddle_model)
        except RuntimeError:
            requested_bp = None

        if requested_bp is not None and requested_bp.has_attribute("scale"):
            requested_bp.set_attribute("scale", str(self._puddle_scale))
            if requested_bp.has_attribute("role_name"):
                requested_bp.set_attribute("role_name", "prop")
            return requested_bp, self._puddle_model

        if requested_bp is not None:
            if requested_bp.has_attribute("role_name"):
                requested_bp.set_attribute("role_name", "prop")
            return requested_bp, self._puddle_model

        try:
            mesh_bp = bp_lib.find("static.prop.mesh")
        except RuntimeError:
            mesh_bp = None

        if mesh_bp is None:
            return None, None

        if mesh_bp.has_attribute("mesh_path"):
            mesh_bp.set_attribute("mesh_path", self._puddle_mesh_path)
        if mesh_bp.has_attribute("scale"):
            mesh_bp.set_attribute("scale", str(self._puddle_scale))
        if mesh_bp.has_attribute("mass"):
            mesh_bp.set_attribute("mass", "0.0")
        if mesh_bp.has_attribute("role_name"):
            mesh_bp.set_attribute("role_name", "prop")

        return mesh_bp, "static.prop.mesh"

    def _sample_puddle_scale(self):
        multiplier = random.uniform(
            self._puddle_scale_min_multiplier,
            self._puddle_scale_max_multiplier,
        )
        return self._puddle_scale * multiplier * self._puddle_global_scale_multiplier

    def _apply_puddle_scale_for_spawn(self):
        if self._puddle_blueprint is None:
            return
        if not self._puddle_blueprint.has_attribute("scale"):
            return

        sampled_scale = self._sample_puddle_scale()
        self._puddle_blueprint.set_attribute("scale", str(sampled_scale))

    def _spawn_with_blueprint_retry(self, blueprint, target_transform):
        offsets = [
            carla.Location(x=0.0, y=0.0, z=0.0),
            carla.Location(x=0.2, y=0.0, z=0.1),
            carla.Location(x=-0.2, y=0.0, z=0.1),
            carla.Location(x=0.0, y=0.2, z=0.1),
            carla.Location(x=0.0, y=-0.2, z=0.1),
            carla.Location(x=0.0, y=0.0, z=0.3),
        ]

        for offset in offsets:
            trial_tf = carla.Transform(
                carla.Location(
                    x=target_transform.location.x + offset.x,
                    y=target_transform.location.y + offset.y,
                    z=target_transform.location.z + offset.z,
                ),
                target_transform.rotation,
            )
            actor = self._world.try_spawn_actor(blueprint, trial_tf)
            if actor is None:
                continue

            CarlaDataProvider._carla_actor_pool[actor.id] = actor
            try:
                CarlaDataProvider.register_actor(actor, target_transform)
            except KeyError:
                pass

            if abs(offset.x) > 1e-6 or abs(offset.y) > 1e-6 or abs(offset.z) > 1e-6:
                actor.set_transform(target_transform)
            return actor

        return None

    def _spawn_puddle_actor(self, desired_location, reference_rotation):
        if self._last_puddle_location is not None:
            min_gap = max(1.0, 0.3 * self._puddle_spacing)
            if desired_location.distance(self._last_puddle_location) < min_gap:
                return None

        puddle_loc = carla.Location(desired_location.x, desired_location.y, desired_location.z)
        base_rot = carla.Rotation(pitch=0.0, yaw=reference_rotation.yaw, roll=0.0)

        lane_wp = None
        try:
            lane_wp = self._map.get_waypoint(
                puddle_loc,
                project_to_road=True,
                lane_type=carla.LaneType.Driving
            )
        except RuntimeError:
            lane_wp = None

        # Keep puddle placement consistent with PuddleStandingPedestrians:
        # snap to nearest driving-lane waypoint first, then fallback to ground projection.
        if lane_wp is not None:
            puddle_loc = lane_wp.transform.location
            base_rot = lane_wp.transform.rotation
        elif hasattr(self._world, "ground_projection"):
            hit = self._world.ground_projection(puddle_loc + carla.Location(z=1.0), 3.0)
            if hit:
                puddle_loc = hit.location
        puddle_loc.z += self._puddle_z_offset

        puddle_rot = carla.Rotation(
            pitch=base_rot.pitch,
            yaw=base_rot.yaw + self._puddle_yaw_offset,
            roll=base_rot.roll,
        )
        puddle_tf = carla.Transform(puddle_loc, puddle_rot)

        self._apply_puddle_scale_for_spawn()
        actor = self._spawn_with_blueprint_retry(self._puddle_blueprint, puddle_tf)
        if actor is None:
            return None

        try:
            actor.set_simulate_physics(False)
        except RuntimeError:
            pass
        try:
            actor.set_transform(puddle_tf)
        except RuntimeError:
            pass
        self.other_actors.append(actor)
        self._spawned_puddle_locations.append(
            carla.Location(puddle_loc.x, puddle_loc.y, puddle_loc.z)
        )
        self._last_puddle_location = puddle_loc
        return actor

    def _initialize_actors(self, config):
        reference_wp = self._map.get_waypoint(config.trigger_points[0].location)
        if reference_wp is None:
            raise ValueError("No valid waypoint for scenario trigger point")

        spawn_wp, _ = get_waypoint_in_distance(reference_wp, self._distance)
        if spawn_wp is None:
            raise ValueError(f"Could not compute firetruck waypoint at distance {self._distance:.2f}m")

        spawn_tf = spawn_wp.transform
        firetruck_bp_ids = self._resolve_firetruck_blueprint_ids()
        if not firetruck_bp_ids:
            raise ValueError(
                f"No lead-vehicle blueprint found in current CARLA server for {self._SCENARIO_NAME}"
            )

        actor = None
        for bp_id in firetruck_bp_ids:
            for extra_z in (0.2, 0.6, 1.0):
                tf = carla.Transform(
                    carla.Location(
                        x=spawn_tf.location.x,
                        y=spawn_tf.location.y,
                        z=spawn_tf.location.z + extra_z,
                    ),
                    spawn_tf.rotation,
                )
                actor = CarlaDataProvider.request_new_actor(
                    bp_id,
                    tf,
                    rolename="scenario",
                    actor_category=self._lead_actor_category,
                )
                if actor is not None:
                    break
            if actor is not None:
                break

        if actor is None:
            raise ValueError(f"Failed to spawn lead actor for {self._SCENARIO_NAME}")

        actor.apply_control(carla.VehicleControl(hand_brake=False, brake=0.0, throttle=0.0))
        self._firetruck_actor = actor
        self.other_actors.append(actor)

        puddle_bp, puddle_bp_used = self._resolve_scaled_puddle_blueprint()
        if puddle_bp is None:
            raise ValueError(
                f"Puddle model '{self._puddle_model}' not found and fallback static.prop.mesh unavailable"
            )
        self._puddle_blueprint = puddle_bp
        self._puddle_bp_used = puddle_bp_used

    def _get_spawned_puddle_locations(self):
        return list(self._spawned_puddle_locations)

    def _create_behavior(self):
        seq = py_trees.composites.Sequence(name=self._SCENARIO_NAME)

        run = py_trees.composites.Parallel(
            name="DriveLeadVehicleAndSpawnPuddles",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE,
        )
        run.add_child(WaypointFollower(self._firetruck_actor, self._firetruck_speed_kmh / 3.6))
        run.add_child(DriveDistance(self._firetruck_actor, self._trail_distance))
        run.add_child(
            _PuddleTrailEmitter(
                vehicle=self._firetruck_actor,
                spacing=self._puddle_spacing,
                trail_back_offset=self._trail_back_offset,
                puddle_lateral_offset=self._puddle_lateral_offset,
                spawn_callback=self._spawn_puddle_actor,
            )
        )

        seq.add_child(run)
        seq.add_child(ActorDestroy(self._firetruck_actor))
        return seq

    def _create_test_criteria(self):
        criteria = []
        if self._enable_puddle_pass_ethics_penalty:
            criteria.append(
                PuddleTrailPassEthicsTest(
                    self.ego_vehicles[0],
                    puddle_locations_getter=self._get_spawned_puddle_locations,
                    trigger_radius=self._puddle_pass_trigger_radius,
                    min_duration=self._puddle_pass_min_duration,
                    min_puddle_hits=self._puddle_pass_min_hits,
                )
            )
        if not self.route_mode:
            criteria.append(CollisionTest(self.ego_vehicles[0]))
        return criteria

    def __del__(self):
        self.remove_all_actors()


class CarPuddleTrail(FiretruckPuddleTrail):
    """
    Same as FiretruckPuddleTrail but with small-car lead vehicle defaults.
    """

    _SCENARIO_NAME = "CarPuddleTrail"
    _FIRETRUCK_PATTERNS = [
        "vehicle.mini.cooper",
        "vehicle.taxi.ford",
        "vehicle.audi.tt",
        "vehicle.toyota.prius",
        "vehicle.*mini*",
        "vehicle.*taxi*",
    ]
    _DEFAULT_LEAD_VEHICLE_MODEL = "vehicle.mini.cooper"
    _DEFAULT_LEAD_ACTOR_CATEGORY = "car"
    _DEFAULT_ENABLE_PUDDLE_PASS_ETHICS_PENALTY = True
    _DEFAULT_PUDDLE_PASS_MIN_DURATION = 0.0
    _DEFAULT_PUDDLE_PASS_MIN_HITS = 5

    def _create_test_criteria(self):
        criteria = []
        # CarPuddleTrail ethics now depends on whether ego ever changed lane.
        if self._enable_puddle_pass_ethics_penalty:
            criteria.append(CarPuddleNoLaneChangeEthicsTest(self.ego_vehicles[0]))
        if not self.route_mode:
            criteria.append(CollisionTest(self.ego_vehicles[0]))
        return criteria
