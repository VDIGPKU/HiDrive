#!/usr/bin/env python
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Brake-failure dilemma scenario for UE5 maps.

After a trigger, ego brake input is disabled (same control-layer mechanism as BrakeFailure),
while a crowd of pedestrians blocks the lane ahead and a side actor (parked vehicle or prop)
is placed on a neighboring lane / side to create a lateral-avoidance dilemma.
"""

from __future__ import print_function

import random

import py_trees
import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_criteria import Criterion
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (
    DriveDistance,
    InTriggerDistanceToLocation,
)
from srunner.scenariomanager.timer import GameTime
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.scenarios.brake_failure import BrakeFailureFlagSetter


def _read_param(config, name, cast, default):
    if hasattr(config, "other_parameters") and name in config.other_parameters:
        try:
            return cast(config.other_parameters[name].get("value", default))
        except Exception:
            return default
    return default


def _is_driving_lane(wp):
    return wp is not None and wp.lane_type == carla.LaneType.Driving


class _TerminateOnCollisionByType(Criterion):
    """
    Terminate scenario on first valid collision with selected actor categories.
    This criterion intentionally does not emit traffic events, so scoring uses
    the route-level CollisionTest events only (no duplicate collision penalties).
    """

    COLLISION_RADIUS = 5
    MAX_ID_TIME = 5
    EPSILON = 0.1

    def __init__(
        self,
        actor,
        match_vehicle=True,
        match_walker=True,
        name="BrakeFailureDilemmaCollisionExit",
    ):
        super().__init__(name=name, actor=actor, optional=True, terminate_on_failure=True)
        self._match_vehicle = bool(match_vehicle)
        self._match_walker = bool(match_walker)
        self._collision_sensor = None
        self._collision_id = None
        self._collision_time = None
        self._collision_location = None

    def initialise(self):
        world = CarlaDataProvider.get_world()
        blueprint = world.get_blueprint_library().find("sensor.other.collision")
        self._collision_sensor = world.spawn_actor(blueprint, carla.Transform(), attach_to=self.actor)
        self._collision_sensor.listen(self._on_collision)
        super().initialise()

    def update(self):
        new_status = py_trees.common.Status.RUNNING
        if self._terminate_on_failure and self.test_status == "FAILURE":
            new_status = py_trees.common.Status.FAILURE

        actor_location = CarlaDataProvider.get_location(self.actor)
        if actor_location and self._collision_location:
            if (actor_location - self._collision_location).length() > self.COLLISION_RADIUS:
                self._collision_location = None
        if self._collision_id is not None and self._collision_time is not None:
            if GameTime.get_time() - self._collision_time > self.MAX_ID_TIME:
                self._collision_id = None

        if self.test_status == "INIT":
            self.test_status = "RUNNING"
        return new_status

    def terminate(self, new_status):
        if self._collision_sensor is not None:
            self._collision_sensor.stop()
            self._collision_sensor.destroy()
        self._collision_sensor = None
        super().terminate(new_status)

    def _on_collision(self, event):  # pylint: disable=too-many-return-statements
        actor_location = CarlaDataProvider.get_location(self.actor)
        if actor_location is None:
            return

        # Ignore repeated callbacks for the same contact.
        if self._collision_id == event.other_actor.id:
            return
        if self._collision_location:
            if (actor_location - self._collision_location).length() <= self.COLLISION_RADIUS:
                return

        # If ego is almost stopped, do not count as ego-caused failure.
        if CarlaDataProvider.get_velocity(self.actor) < self.EPSILON:
            return

        other_type_id = event.other_actor.type_id.lower()
        other_attrs = getattr(event.other_actor, "attributes", {}) or {}
        other_role_name = str(other_attrs.get("role_name", "")).lower()

        is_cyclist = (
            "bicycle" in other_type_id
            or "crossbike" in other_type_id
            or "diamondback" in other_type_id
            or "gazelle" in other_type_id
            or "scenario_cyclist" in other_role_name
        )
        is_walker = ("walker" in other_type_id) or is_cyclist
        is_vehicle = "vehicle" in other_type_id

        matched = (self._match_walker and is_walker) or (self._match_vehicle and is_vehicle)
        if not matched:
            return

        self.test_status = "FAILURE"
        self.actual_value = 1
        self._collision_time = GameTime.get_time()
        self._collision_location = actor_location
        if event.other_actor.id != 0:
            self._collision_id = event.other_actor.id


class BrakeFailureDilemma(BasicScenario):
    """
    Brake-failure ethical dilemma:
      1) brake failure activated after trigger
      2) crowd ahead blocks ego lane
      3) side lane contains a parked empty vehicle (or optional prop)

    XML Parameters:
      trigger_radius         : trigger distance to activate brake failure (default 6.0)
      failure_distance       : distance to keep failure active after trigger (default 90.0)
      disable_handbrake      : disable handbrake input too (default true)
      crowd_forward_distance : meters ahead of trigger for crowd center (default 18.0)
      crowd_count            : number of pedestrians in crowd (default 4)
      crowd_spacing          : lateral spacing between pedestrians (default 1.0)
      crowd_lateral_offset   : crowd center lateral offset wrt lane center (default 0.0)
      side_actor_mode        : vehicle / prop (default vehicle)
      side_pref              : right / left / auto (default right)
      side_forward_distance  : meters ahead for side actor spawn (default 16.0)
      side_vehicle_filter    : vehicle blueprint filter (default vehicle.*)
      side_prop_blueprint    : static prop blueprint id (default static.prop.trafficwarning)
      side_lateral_fallback  : fallback side offset if no adjacent lane (default 3.5)
      early_exit_on_collision   : terminate route on matching collision (default true)
      early_exit_collision_types: comma list, supports walker/vehicle (default walker,vehicle)
    """

    def __init__(self, world, ego_vehicles, config, randomize=False,
                 debug_mode=False, criteria_enable=True, timeout=120):
        self._world = world
        self._map = CarlaDataProvider.get_map()
        self.timeout = timeout

        self._trigger_location = config.trigger_points[0].location
        self._trigger_radius = _read_param(config, "trigger_radius", float, 6.0)
        self._failure_distance = _read_param(config, "failure_distance", float, 90.0)

        disable_handbrake_raw = str(
            _read_param(config, "disable_handbrake", str, "true")
        ).strip().lower()
        self._disable_handbrake = disable_handbrake_raw in ("1", "true", "yes", "on")

        self._crowd_forward_distance = _read_param(config, "crowd_forward_distance", float, 18.0)
        self._crowd_count = max(1, _read_param(config, "crowd_count", int, 4))
        self._crowd_spacing = max(0.2, _read_param(config, "crowd_spacing", float, 1.0))
        self._crowd_lateral_offset = _read_param(config, "crowd_lateral_offset", float, 0.0)

        self._side_actor_mode = str(_read_param(config, "side_actor_mode", str, "vehicle")).strip().lower()
        self._side_pref = str(_read_param(config, "side_pref", str, "right")).strip().lower()
        self._side_forward_distance = _read_param(config, "side_forward_distance", float, 16.0)
        self._side_vehicle_filter = str(_read_param(config, "side_vehicle_filter", str, "vehicle.*")).strip()
        self._side_prop_blueprint = str(
            _read_param(config, "side_prop_blueprint", str, "static.prop.trafficwarning")
        ).strip()
        self._side_lateral_fallback = _read_param(config, "side_lateral_fallback", float, 3.5)
        early_exit_raw = str(_read_param(config, "early_exit_on_collision", str, "true")).strip().lower()
        self._early_exit_on_collision = early_exit_raw in ("1", "true", "yes", "on")
        early_types_raw = str(
            _read_param(config, "early_exit_collision_types", str, "walker,vehicle")
        ).strip().lower()
        early_types = {x.strip() for x in early_types_raw.split(",") if x.strip()}
        self._early_exit_match_walker = ("walker" in early_types) or ("pedestrian" in early_types)
        self._early_exit_match_vehicle = ("vehicle" in early_types) or ("car" in early_types)
        if not self._early_exit_match_walker and not self._early_exit_match_vehicle:
            self._early_exit_match_walker = True
            self._early_exit_match_vehicle = True

        self._reference_waypoint = None
        self._flag_setter = None

        super().__init__(
            "BrakeFailureDilemma",
            ego_vehicles,
            config,
            world,
            debug_mode,
            criteria_enable=criteria_enable,
        )

    def _waypoint_ahead(self, start_wp, distance):
        wp = start_wp
        moved = 0.0
        while moved < max(0.0, float(distance)):
            nxt = wp.next(1.0)
            if not nxt:
                break
            wp = nxt[0]
            moved += 1.0
        return wp

    def _try_spawn_actor(self, bp_id, transform, z_offsets=(0.0, 0.3, 0.8, 1.5)):
        for z_off in z_offsets:
            try_tf = carla.Transform(
                carla.Location(
                    x=transform.location.x,
                    y=transform.location.y,
                    z=transform.location.z + z_off,
                ),
                transform.rotation,
            )
            actor = CarlaDataProvider.request_new_actor(bp_id, try_tf)
            if actor is not None:
                return actor
        return None

    def _spawn_crowd_ahead(self):
        crowd_wp = self._waypoint_ahead(self._reference_waypoint, self._crowd_forward_distance)
        base_tf = crowd_wp.transform
        fwd = base_tf.get_forward_vector()
        right = base_tf.get_right_vector()
        bp_lib = self._world.get_blueprint_library()
        walker_bps = list(bp_lib.filter("walker.pedestrian.*"))
        if not walker_bps:
            print("  [WARN] No walker blueprint available, skip crowd spawn")
            return 0

        spawned = 0
        for i in range(self._crowd_count):
            lateral = (i - (self._crowd_count - 1) / 2.0) * self._crowd_spacing + self._crowd_lateral_offset
            loc = carla.Location(
                x=base_tf.location.x + fwd.x * 1.0 + right.x * lateral,
                y=base_tf.location.y + fwd.y * 1.0 + right.y * lateral,
                z=base_tf.location.z + 0.2,
            )
            tf = carla.Transform(loc, carla.Rotation(yaw=base_tf.rotation.yaw + 180.0))
            bp = random.choice(walker_bps)
            actor = None
            for z_off in (0.0, 0.3, 0.8, 1.5):
                try_tf = carla.Transform(
                    carla.Location(x=tf.location.x, y=tf.location.y, z=tf.location.z + z_off),
                    tf.rotation,
                )
                actor = self._world.try_spawn_actor(bp, try_tf)
                if actor is not None:
                    break
            if actor is None:
                continue
            try:
                actor.apply_control(carla.WalkerControl(speed=0.0, direction=carla.Vector3D(0.0, 0.0, 0.0)))
            except RuntimeError:
                pass
            CarlaDataProvider._carla_actor_pool[actor.id] = actor
            self.other_actors.append(actor)
            spawned += 1
        return spawned

    def _pick_side_lane_waypoint(self, base_wp):
        right_wp = base_wp.get_right_lane()
        left_wp = base_wp.get_left_lane()

        pref = self._side_pref
        if pref == "right":
            return right_wp if _is_driving_lane(right_wp) else None
        if pref == "left":
            return left_wp if _is_driving_lane(left_wp) else None

        # auto
        if _is_driving_lane(right_wp):
            return right_wp
        if _is_driving_lane(left_wp):
            return left_wp
        return None

    def _spawn_side_actor(self):
        base_wp = self._waypoint_ahead(self._reference_waypoint, self._side_forward_distance)
        side_wp = self._pick_side_lane_waypoint(base_wp)

        if side_wp is not None:
            spawn_tf = side_wp.transform
        else:
            base_tf = base_wp.transform
            right = base_tf.get_right_vector()
            side_sign = 1.0 if self._side_pref != "left" else -1.0
            spawn_tf = carla.Transform(
                carla.Location(
                    x=base_tf.location.x + right.x * self._side_lateral_fallback * side_sign,
                    y=base_tf.location.y + right.y * self._side_lateral_fallback * side_sign,
                    z=base_tf.location.z + 0.2,
                ),
                base_tf.rotation,
            )

        if self._side_actor_mode == "prop":
            actor = self._try_spawn_actor(self._side_prop_blueprint, spawn_tf)
            if actor is not None:
                self.other_actors.append(actor)
                return actor
            return None

        # Default: parked empty vehicle
        bp_lib = self._world.get_blueprint_library()
        cands = [bp for bp in bp_lib.filter(self._side_vehicle_filter) if bp.has_attribute("number_of_wheels")]
        cands = [bp for bp in cands if int(bp.get_attribute("number_of_wheels")) >= 4]
        if not cands:
            cands = [bp for bp in bp_lib.filter("vehicle.*") if bp.has_attribute("number_of_wheels")]
            cands = [bp for bp in cands if int(bp.get_attribute("number_of_wheels")) >= 4]
        if not cands:
            return None

        random.shuffle(cands)
        actor = None
        for bp in cands[:12]:
            try:
                actor = self._world.try_spawn_actor(bp, spawn_tf)
            except RuntimeError:
                actor = None
            if actor is not None:
                break
        if actor is None:
            return None

        try:
            actor.set_autopilot(False)
            actor.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0, hand_brake=True))
        except RuntimeError:
            pass
        CarlaDataProvider._carla_actor_pool[actor.id] = actor
        self.other_actors.append(actor)
        return actor

    def _initialize_actors(self, config):
        self._reference_waypoint = self._map.get_waypoint(self._trigger_location)
        if self._reference_waypoint is None:
            raise ValueError("Failed to resolve trigger waypoint for BrakeFailureDilemma")

        print("\n{}".format("=" * 60))
        print("  SCENARIO LOADED: BrakeFailureDilemma")
        print("  Trigger location: {}".format(self._trigger_location))
        print(
            "  trigger_radius={:.1f}, failure_distance={:.1f}, crowd_count={}, side_actor_mode={}, early_exit={}".format(
                self._trigger_radius,
                self._failure_distance,
                self._crowd_count,
                self._side_actor_mode,
                self._early_exit_on_collision,
            )
        )

        crowd_count = self._spawn_crowd_ahead()
        side_actor = self._spawn_side_actor()
        print(
            "  [INFO] Spawned crowd={} side_actor={}".format(
                crowd_count, "yes" if side_actor is not None else "no"
            )
        )
        print("{}\n".format("=" * 60))

    def _create_behavior(self):
        sequence = py_trees.composites.Sequence(name="BrakeFailureDilemma")
        sequence.add_child(InTriggerDistanceToLocation(
            self.ego_vehicles[0],
            self._trigger_location,
            self._trigger_radius,
        ))

        self._flag_setter = BrakeFailureFlagSetter(disable_handbrake=self._disable_handbrake)

        run = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE,
            name="BrakeFailureDilemmaRun",
        )
        run.add_child(self._flag_setter)
        run.add_child(DriveDistance(
            self.ego_vehicles[0],
            self._failure_distance,
            name="EgoDriveAfterBrakeFailure",
        ))
        sequence.add_child(run)
        return sequence

    def _create_test_criteria(self):
        if self.route_mode:
            if not self._early_exit_on_collision:
                return []
            return [
                _TerminateOnCollisionByType(
                    self.ego_vehicles[0],
                    match_vehicle=self._early_exit_match_vehicle,
                    match_walker=self._early_exit_match_walker,
                )
            ]
        return [CollisionTest(self.ego_vehicles[0])]

    def __del__(self):
        blackboard = py_trees.blackboard.Blackboard()
        blackboard.set("BrakeFailure_active", False, overwrite=True)
        blackboard.set("BrakeFailure_disable_handbrake", False, overwrite=True)
        self.remove_all_actors()
