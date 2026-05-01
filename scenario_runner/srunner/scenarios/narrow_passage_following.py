#!/usr/bin/env python

"""
Narrow passage following scenario:

Combines NarrowPassage obstacles with lead/rear vehicles to test car-following
behavior in constrained spaces. The lead vehicle drives through the narrow
passage and then suddenly brakes. The rear vehicle (if present) brakes with
a configurable delay after the lead vehicle brakes.

Sub-scenarios:
  1. NarrowPassageFollowingFront  - lead vehicle only, no rear vehicle
  2. NarrowPassageFollowingBoth   - lead + rear vehicle

Goal: test whether ego maintains safe following distance and brakes in time.

Key design: vehicles follow an offset path (passage centre) rather than the
lane centre, so they won't collide with the narrow-passage obstacles even
when lateral_offset != 0.
"""

import math
import py_trees
import carla
import time

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import Idle, ActorDestroy
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenarios.basic_scenario import BasicScenario


# ---------------------------------------------------------------------------
#  Helper: build offset path from waypoints
# ---------------------------------------------------------------------------

def _offset_location(wp, lateral_offset):
    """Return a carla.Location shifted from *wp* by *lateral_offset* along
    the waypoint's right vector (positive = right, negative = left)."""
    rv = wp.transform.get_right_vector()
    loc = wp.transform.location
    return carla.Location(
        x=loc.x + lateral_offset * rv.x,
        y=loc.y + lateral_offset * rv.y,
        z=loc.z)

# PLACEHOLDER_REST -- build_offset_path and behaviors

def _build_offset_path(waypoints, lateral_offset):
    """Given a list of carla.Waypoint, return a list of carla.Transform
    shifted by lateral_offset.  Each transform keeps the original rotation
    so the vehicle heading stays aligned with the road."""
    path = []
    for wp in waypoints:
        loc = _offset_location(wp, lateral_offset)
        path.append(carla.Transform(loc, wp.transform.rotation))
    return path


def _lead_spawn_transform_from_trigger(map_ref, trigger_location, lateral_offset,
                                       lead_distance, lead_trigger_extra,
                                       forward_distance):
    """Compute lead spawn transform from trigger point.

    Spawn distance is (lead_distance + lead_trigger_extra), clamped so the
    lead vehicle stays before the first obstacle pair.
    """
    trigger_wp = map_ref.get_waypoint(trigger_location)
    if trigger_wp is None:
        raise RuntimeError("Cannot find trigger waypoint for lead vehicle")

    requested = max(2.0, float(lead_distance) + float(lead_trigger_extra))
    # Obstacles start at forward_distance from trigger; keep a small margin.
    max_before_obstacle = max(2.0, float(forward_distance) - 2.0)
    spawn_distance = min(requested, max_before_obstacle)

    lead_wps = trigger_wp.next(spawn_distance)
    if not lead_wps:
        raise RuntimeError("Cannot find waypoint ahead from trigger for lead vehicle")

    lead_wp = lead_wps[0]
    lead_loc = _offset_location(lead_wp, lateral_offset)
    return carla.Transform(lead_loc, lead_wp.transform.rotation)


def _safe_disable_autopilot(actor):
    """Best-effort autopilot disable.

    In some UE5/TM states, calling set_autopilot(False) on an actor that is
    not managed by TM can raise RuntimeError(std::exception). This scenario
    controls vehicles via constant velocity / apply_control, so autopilot
    disable is optional and should not crash the route.
    """
    if actor is None:
        return
    if hasattr(actor, "is_alive") and not actor.is_alive:
        return
    try:
        actor.set_autopilot(False)
    except RuntimeError:
        pass


# ---------------------------------------------------------------------------
#  Helper behaviors
# ---------------------------------------------------------------------------

class OffsetPathFollower(py_trees.behaviour.Behaviour):
    """Drive an actor along a list of carla.Transform at *target_speed* (m/s).

    Uses a simple pure-pursuit-like controller:
    - Steer toward the next target point
    - PID-ish throttle/brake to maintain target speed

    Returns RUNNING while driving, SUCCESS when the plan is exhausted.
    """

    LOOK_AHEAD = 4.0   # metres – switch to next point when closer than this

    def __init__(self, actor, path, target_speed, name="OffsetPathFollower"):
        super().__init__(name)
        self._actor = actor
        self._path = list(path)   # list of carla.Transform
        self._target_speed = target_speed
        self._idx = 0
        self._aligned_start_idx = False

    def _pick_start_index(self):
        """Choose a robust start index on first tick.

        Prefer the nearest point that is in front of the actor (to avoid
        steering toward points behind the vehicle when spawn timing varies).
        Fall back to the globally nearest point if no "in-front" point exists.
        """
        if not self._path:
            return 0

        loc = self._actor.get_location()
        fwd = self._actor.get_transform().get_forward_vector()

        best_any_idx, best_any_d2 = 0, float('inf')
        best_front_idx, best_front_d2 = None, float('inf')

        for i, tf in enumerate(self._path):
            dx = tf.location.x - loc.x
            dy = tf.location.y - loc.y
            d2 = dx * dx + dy * dy

            if d2 < best_any_d2:
                best_any_idx, best_any_d2 = i, d2

            dot = dx * fwd.x + dy * fwd.y
            if dot >= -1.0 and d2 < best_front_d2:
                best_front_idx, best_front_d2 = i, d2

        return best_front_idx if best_front_idx is not None else best_any_idx

    def update(self):
        # First tick: disable constant velocity so apply_control works
        if not hasattr(self, '_took_over'):
            self._took_over = True
            try:
                self._actor.disable_constant_velocity()
            except RuntimeError:
                pass

        # Align to nearest forward path point once, avoids lateral drift at spawn
        # when actor starts ahead of path[0].
        if not self._aligned_start_idx:
            self._idx = self._pick_start_index()
            self._aligned_start_idx = True

        if self._idx >= len(self._path):
            return py_trees.common.Status.SUCCESS

        # Advance index if close enough
        loc = self._actor.get_location()
        while self._idx < len(self._path):
            target_loc = self._path[self._idx].location
            if loc.distance(target_loc) < self.LOOK_AHEAD:
                self._idx += 1
            else:
                break
        if self._idx >= len(self._path):
            return py_trees.common.Status.SUCCESS

        target_loc = self._path[self._idx].location

        # --- Steering ---
        dx = target_loc.x - loc.x
        dy = target_loc.y - loc.y
        target_yaw = math.atan2(dy, dx)

        actor_yaw = math.radians(self._actor.get_transform().rotation.yaw)
        err = target_yaw - actor_yaw
        # Normalise to [-pi, pi]
        err = (err + math.pi) % (2 * math.pi) - math.pi
        steer = max(-1.0, min(1.0, err * 2.0))

        # --- Throttle / brake ---
        vel = self._actor.get_velocity()
        speed = math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)

        control = carla.VehicleControl()
        control.steer = steer
        if speed < self._target_speed * 0.9:
            control.throttle = min(1.0, 0.5 + (self._target_speed - speed) * 0.3)
            control.brake = 0.0
        elif speed > self._target_speed * 1.1:
            control.throttle = 0.0
            control.brake = 0.3
        else:
            control.throttle = 0.3
            control.brake = 0.0
        self._actor.apply_control(control)
        return py_trees.common.Status.RUNNING




class TimedOffsetPathFollower(py_trees.behaviour.Behaviour):
    """Run OffsetPathFollower for a fixed duration, then succeed.

    This is used to guarantee actors are destroyed exactly N seconds after
    resuming movement, independent of whether they reach the end of path.
    """

    def __init__(self, actor, path, target_speed, duration,
                 name="TimedOffsetPathFollower"):
        super().__init__(name)
        self._duration = max(0.0, float(duration))
        self._follower = OffsetPathFollower(
            actor, path, target_speed, name=f"{name}_Follower")
        self._start_time = None

    def update(self):
        if self._start_time is None:
            self._start_time = time.time()

        # Keep driving while the timer runs.
        self._follower.update()

        if (time.time() - self._start_time) >= self._duration:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING

class EmergencyBrake(py_trees.behaviour.Behaviour):
    """Disable autopilot and apply full brake until the vehicle stops."""

    def __init__(self, actor, speed_threshold=0.3, name="EmergencyBrake"):
        super().__init__(name)
        self._actor = actor
        self._speed_threshold = speed_threshold
        self._started = False

    def update(self):
        if not self._started:
            try:
                self._actor.disable_constant_velocity()
            except RuntimeError:
                pass
            _safe_disable_autopilot(self._actor)
            self._started = True

        control = carla.VehicleControl()
        control.brake = 1.0
        control.throttle = 0.0
        control.steer = 0.0
        self._actor.apply_control(control)

        lights = self._actor.get_light_state()
        lights |= carla.VehicleLightState.Brake
        self._actor.set_light_state(carla.VehicleLightState(lights))

        vel = self._actor.get_velocity()
        speed = math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)
        if speed < self._speed_threshold:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING


# PLACEHOLDER_DELAYED -- DelayedEmergencyBrake, WaitForActorDistance, obstacle helper

class DelayedEmergencyBrake(py_trees.behaviour.Behaviour):
    """Wait *delay* seconds, then brake with *brake_force* until stopped.
    brake_force < 1.0 produces a softer brake with longer sliding distance.
    Used for the rear vehicle to simulate delayed reaction."""

    def __init__(self, actor, delay=1.0, brake_force=1.0,
                 speed_threshold=0.3, name="DelayedEmergencyBrake"):
        super().__init__(name)
        self._actor = actor
        self._delay = delay
        self._brake_force = max(0.1, min(1.0, brake_force))
        self._speed_threshold = speed_threshold
        self._start_time = None
        self._braking = False

    def update(self):
        if self._start_time is None:
            self._start_time = time.time()

        elapsed = time.time() - self._start_time
        if not self._braking and elapsed < self._delay:
            return py_trees.common.Status.RUNNING

        if not self._braking:
            try:
                self._actor.disable_constant_velocity()
            except RuntimeError:
                pass
            _safe_disable_autopilot(self._actor)
            self._braking = True

        control = carla.VehicleControl()
        control.brake = self._brake_force
        control.throttle = 0.0
        control.steer = 0.0
        self._actor.apply_control(control)

        lights = self._actor.get_light_state()
        lights |= carla.VehicleLightState.Brake
        self._actor.set_light_state(carla.VehicleLightState(lights))

        vel = self._actor.get_velocity()
        speed = math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)
        if speed < self._speed_threshold:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING


class WaitForActorNearLocation(py_trees.behaviour.Behaviour):
    """RUNNING until *actor* is within *distance* metres of *location*."""

    def __init__(self, actor, location, distance, name="WaitForActorNearLocation"):
        super().__init__(name)
        self._actor = actor
        self._location = location
        self._distance = distance

    def update(self):
        if self._actor.get_location().distance(self._location) < self._distance:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING


class ReEnableConstantVelocity(py_trees.behaviour.Behaviour):
    """Re-enable constant velocity on an actor (one-shot, returns SUCCESS)."""

    def __init__(self, actor, speed, name="ReEnableConstantVelocity"):
        super().__init__(name)
        self._actor = actor
        self._speed = speed

    def update(self):
        fwd = self._actor.get_transform().get_forward_vector()
        self._actor.enable_constant_velocity(carla.Vector3D(
            x=fwd.x * self._speed,
            y=fwd.y * self._speed,
            z=0))
        return py_trees.common.Status.SUCCESS


# ---------------------------------------------------------------------------
#  Obstacle placement helper
# ---------------------------------------------------------------------------

def _place_narrow_obstacles(world, map_ref, trigger_location,
                            obstacle_type, num_pairs, pair_spacing,
                            gap_width, forward_distance, lateral_offset):
    """Place obstacle pairs and return (actors, passage_waypoints).

    passage_waypoints are the raw carla.Waypoint at each pair + beyond.
    The caller should use _build_offset_path() to shift them for vehicle driving.
    """
    ref_wp = map_ref.get_waypoint(trigger_location)
    bp_name = f'static.prop.{obstacle_type}'
    bp_lib = world.get_blueprint_library()
    bp_list = list(bp_lib.filter(bp_name))
    if not bp_list:
        raise RuntimeError(f"Blueprint '{bp_name}' not found")
    blueprint = bp_list[0]

    # Walk forward to first pair
    wp = ref_wp
    remaining = forward_distance
    while remaining > 0:
        nxt = wp.next(min(remaining, 2.0))
        if not nxt:
            break
        wp = nxt[0]
        remaining -= 2.0

    actors = []
    passage_wps = []

    for i in range(num_pairs):
        right_vec = wp.transform.get_right_vector()
        centre = wp.transform.location
        half_gap = gap_width / 2.0

        cx = centre.x + lateral_offset * right_vec.x
        cy = centre.y + lateral_offset * right_vec.y

        left_loc = carla.Location(
            x=cx - half_gap * right_vec.x,
            y=cy - half_gap * right_vec.y,
            z=centre.z + 0.3)
        left_actor = world.try_spawn_actor(
            blueprint, carla.Transform(left_loc, wp.transform.rotation))
        if left_actor:
            actors.append(left_actor)
            CarlaDataProvider._carla_actor_pool[left_actor.id] = left_actor

        right_loc = carla.Location(
            x=cx + half_gap * right_vec.x,
            y=cy + half_gap * right_vec.y,
            z=centre.z + 0.3)
        right_actor = world.try_spawn_actor(
            blueprint, carla.Transform(right_loc, wp.transform.rotation))
        if right_actor:
            actors.append(right_actor)
            CarlaDataProvider._carla_actor_pool[right_actor.id] = right_actor

        passage_wps.append(wp)

        nxt = wp.next(pair_spacing)
        if not nxt:
            break
        wp = nxt[0]

    # Extra waypoints beyond obstacles for vehicles to drive past
    for _ in range(10):
        nxt = wp.next(3.0)
        if not nxt:
            break
        wp = nxt[0]
        passage_wps.append(wp)

    print(f"  [NarrowPassageFollowing] Placed {len(actors)} obstacles "
          f"({num_pairs} pairs, gap={gap_width}m, offset={lateral_offset}m)")
    return actors, passage_wps


# ---------------------------------------------------------------------------
#  Sub-scenario 1: Lead vehicle only (front, no rear)
# ---------------------------------------------------------------------------

class NarrowPassageFollowingFront(BasicScenario):
    """
    Narrow passage with a lead vehicle that drives through the passage centre
    and then emergency-brakes. Tests ego following distance.

    The lead vehicle follows an offset path matching the passage centre
    (lane_centre + lateral_offset), so it won't hit the obstacles.
    """

    def __init__(self, world, ego_vehicles, config, randomize=False,
                 debug_mode=False, criteria_enable=True, timeout=180):
        self._world = world
        self._map = CarlaDataProvider.get_map()
        self.timeout = timeout

        p = config.other_parameters
        self._obstacle_type = p.get('obstacle_type', {}).get('value', 'warningconstruction')
        self._num_pairs = int(p.get('num_pairs', {}).get('value', 4))
        self._pair_spacing = float(p.get('pair_spacing', {}).get('value', 6))
        self._gap_width = float(p.get('gap_width', {}).get('value', 4))
        self._forward_distance = float(p.get('forward_distance', {}).get('value', 15))
        self._lateral_offset = float(p.get('lateral_offset', {}).get('value', -1))
        self._lead_distance = float(p.get('lead_distance', {}).get('value', 12))
        self._lead_trigger_extra = float(p.get('lead_trigger_extra', {}).get('value', 3.0))
        self._lead_speed = float(p.get('lead_speed', {}).get('value', 6))
        self._brake_after_pairs = int(p.get('brake_after_pairs', {}).get('value', self._num_pairs))
        self._stop_duration = float(p.get('stop_duration', {}).get('value', 15))

        self._trigger_location = config.trigger_points[0].location
        self._passage_wps = []
        self._offset_path = []

        super().__init__("NarrowPassageFollowingFront",
                         ego_vehicles, config, world,
                         debug_mode, criteria_enable=criteria_enable)

    def _initialize_actors(self, config):
        # 1. Place obstacles
        obs_actors, self._passage_wps = _place_narrow_obstacles(
            self._world, self._map, self._trigger_location,
            self._obstacle_type, self._num_pairs, self._pair_spacing,
            self._gap_width, self._forward_distance, self._lateral_offset)
        for a in obs_actors:
            self.other_actors.append(a)

        # 2. Build offset driving path (passage centre)
        self._offset_path = _build_offset_path(self._passage_wps, self._lateral_offset)

        # 3. Spawn lead vehicle slightly farther from trigger point
        lead_transform = _lead_spawn_transform_from_trigger(
            self._map, self._trigger_location, self._lateral_offset,
            self._lead_distance, self._lead_trigger_extra,
            self._forward_distance)

        self._lead_vehicle = CarlaDataProvider.request_new_actor(
            'vehicle.*', lead_transform, rolename='scenario',
            attribute_filter={'base_type': 'car', 'has_lights': True})
        self.other_actors.append(self._lead_vehicle)
        self._lead_vehicle.set_simulate_physics(True)

        # Keep stable until the behavior tree takes over.
        self._lead_vehicle.apply_control(carla.VehicleControl(
            throttle=0.0, brake=1.0, steer=0.0))

    def _create_behavior(self):
        root = py_trees.composites.Sequence("NarrowPassageFollowingFront")

        # Phase 1: Lead drives along offset path until it reaches brake point
        lead_drive = OffsetPathFollower(
            self._lead_vehicle, self._offset_path, self._lead_speed)

        brake_idx = min(self._brake_after_pairs, len(self._offset_path) - 1)
        brake_location = self._offset_path[brake_idx].location

        phase1 = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE,
            name="LeadDriveToBrakePoint")
        phase1.add_child(lead_drive)
        phase1.add_child(WaitForActorNearLocation(
            self._lead_vehicle, brake_location, 5.0))
        root.add_child(phase1)

        # Phase 2: Emergency brake
        root.add_child(EmergencyBrake(self._lead_vehicle))

        # Phase 3: Hold stopped for configured stop_duration
        root.add_child(Idle(self._stop_duration))

        # Phase 4: Lead resumes for 5s
        remaining_path = self._offset_path[brake_idx:]
        resume_path = remaining_path if remaining_path else self._offset_path
        root.add_child(TimedOffsetPathFollower(
            self._lead_vehicle, resume_path, self._lead_speed, duration=5.0,
            name="LeadResumeFor5s"))

        # Phase 5: Destroy lead vehicle
        root.add_child(ActorDestroy(self._lead_vehicle))
        return root

    def _create_test_criteria(self):
        if self.route_mode:
            return []
        return [CollisionTest(self.ego_vehicles[0])]

    def __del__(self):
        self.remove_all_actors()


# ---------------------------------------------------------------------------
#  Sub-scenario 2: Lead + Rear vehicles
# ---------------------------------------------------------------------------

class NarrowPassageFollowingBoth(BasicScenario):
    """
    Narrow passage with lead AND rear vehicles.
    Lead drives through passage centre and emergency-brakes; rear vehicle
    brakes with a configurable delay. Tests ego gap management on both sides.

    The rear vehicle also follows the offset path so it won't hit obstacles.
    """

    def __init__(self, world, ego_vehicles, config, randomize=False,
                 debug_mode=False, criteria_enable=True, timeout=180):
        self._world = world
        self._map = CarlaDataProvider.get_map()
        self.timeout = timeout

        p = config.other_parameters
        self._obstacle_type = p.get('obstacle_type', {}).get('value', 'warningconstruction')
        self._num_pairs = int(p.get('num_pairs', {}).get('value', 4))
        self._pair_spacing = float(p.get('pair_spacing', {}).get('value', 6))
        self._gap_width = float(p.get('gap_width', {}).get('value', 4))
        self._forward_distance = float(p.get('forward_distance', {}).get('value', 15))
        self._lateral_offset = float(p.get('lateral_offset', {}).get('value', -1))
        self._lead_distance = float(p.get('lead_distance', {}).get('value', 12))
        self._lead_trigger_extra = float(p.get('lead_trigger_extra', {}).get('value', 3.0))
        self._lead_speed = float(p.get('lead_speed', {}).get('value', 6))
        self._brake_after_pairs = int(p.get('brake_after_pairs', {}).get('value', self._num_pairs))
        self._stop_duration = float(p.get('stop_duration', {}).get('value', 15))
        self._rear_distance = float(p.get('rear_distance', {}).get('value', 12))
        self._rear_speed = float(p.get('rear_speed', {}).get('value', 7))
        self._rear_brake_delay = float(p.get('rear_brake_delay', {}).get('value', 1.5))
        self._rear_brake_force = float(p.get('rear_brake_force', {}).get('value', 0.5))

        self._trigger_location = config.trigger_points[0].location
        self._passage_wps = []
        self._offset_path = []

        super().__init__("NarrowPassageFollowingBoth",
                         ego_vehicles, config, world,
                         debug_mode, criteria_enable=criteria_enable)

    def _initialize_actors(self, config):
        # 1. Place obstacles
        obs_actors, self._passage_wps = _place_narrow_obstacles(
            self._world, self._map, self._trigger_location,
            self._obstacle_type, self._num_pairs, self._pair_spacing,
            self._gap_width, self._forward_distance, self._lateral_offset)
        for a in obs_actors:
            self.other_actors.append(a)

        # 2. Build offset path
        self._offset_path = _build_offset_path(self._passage_wps, self._lateral_offset)

        ego_wp = self._map.get_waypoint(self.ego_vehicles[0].get_location())

        # 3. Spawn lead vehicle slightly farther from trigger point
        lead_transform = _lead_spawn_transform_from_trigger(
            self._map, self._trigger_location, self._lateral_offset,
            self._lead_distance, self._lead_trigger_extra,
            self._forward_distance)
        self._lead_vehicle = CarlaDataProvider.request_new_actor(
            'vehicle.*', lead_transform, rolename='scenario',
            attribute_filter={'base_type': 'car', 'has_lights': True})
        self.other_actors.append(self._lead_vehicle)
        self._lead_vehicle.set_simulate_physics(True)

        # Keep stable until the behavior tree takes over.
        self._lead_vehicle.apply_control(carla.VehicleControl(
            throttle=0.0, brake=1.0, steer=0.0))

        # 4. Spawn rear vehicle behind ego, facing FORWARD
        rear_wps = ego_wp.previous(self._rear_distance)
        if not rear_wps:
            raise RuntimeError("Cannot find waypoint behind for rear vehicle")
        rear_loc = _offset_location(rear_wps[0], self._lateral_offset)
        # Use ego's rotation (forward), not previous()'s backward rotation
        rear_transform = carla.Transform(rear_loc, ego_wp.transform.rotation)
        self._rear_vehicle = CarlaDataProvider.request_new_actor(
            'vehicle.*', rear_transform, rolename='scenario',
            attribute_filter={'base_type': 'car', 'has_lights': True})
        self.other_actors.append(self._rear_vehicle)
        self._rear_vehicle.set_simulate_physics(True)

        # Keep stable until the behavior tree takes over.
        self._rear_vehicle.apply_control(carla.VehicleControl(
            throttle=0.0, brake=1.0, steer=0.0))

    def _create_behavior(self):
        root = py_trees.composites.Sequence("NarrowPassageFollowingBoth")

        # Lead + rear both drive along offset path
        lead_drive = OffsetPathFollower(
            self._lead_vehicle, self._offset_path, self._lead_speed)
        rear_drive = OffsetPathFollower(
            self._rear_vehicle, self._offset_path, self._rear_speed)

        brake_idx = min(self._brake_after_pairs, len(self._offset_path) - 1)
        brake_location = self._offset_path[brake_idx].location

        # Phase 1: both drive; ends when lead reaches brake point
        drive_both = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL,
            name="BothDrive")
        drive_both.add_child(lead_drive)
        drive_both.add_child(rear_drive)

        phase1 = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE,
            name="DriveUntilBrakePoint")
        phase1.add_child(drive_both)
        phase1.add_child(WaitForActorNearLocation(
            self._lead_vehicle, brake_location, 5.0))
        root.add_child(phase1)

        # Phase 2: Lead brakes immediately + rear brakes with delay
        brake_phase = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL,
            name="BothBrake")
        brake_phase.add_child(EmergencyBrake(self._lead_vehicle))
        brake_phase.add_child(DelayedEmergencyBrake(
            self._rear_vehicle, delay=self._rear_brake_delay,
            brake_force=self._rear_brake_force))
        root.add_child(brake_phase)

        # Phase 3: Hold stopped for configured stop_duration
        root.add_child(Idle(self._stop_duration))

        # Phase 4: Both resume for 5s
        remaining_lead = self._offset_path[brake_idx:]
        lead_resume_path = remaining_lead if remaining_lead else self._offset_path
        resume_phase = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL,
            name="BothResumeFor5s")
        resume_phase.add_child(TimedOffsetPathFollower(
            self._lead_vehicle, lead_resume_path, self._lead_speed,
            duration=5.0, name="LeadResumeFor5s"))
        resume_phase.add_child(TimedOffsetPathFollower(
            self._rear_vehicle, self._offset_path, self._rear_speed,
            duration=5.0, name="RearResumeFor5s"))
        root.add_child(resume_phase)

        # Phase 5: Destroy lead and rear vehicles
        root.add_child(ActorDestroy(self._lead_vehicle))
        root.add_child(ActorDestroy(self._rear_vehicle))
        return root

    def _create_test_criteria(self):
        if self.route_mode:
            return []
        return [CollisionTest(self.ego_vehicles[0])]

    def __del__(self):
        self.remove_all_actors()
