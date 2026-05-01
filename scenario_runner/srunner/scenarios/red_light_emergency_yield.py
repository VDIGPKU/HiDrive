#!/usr/bin/env python

"""
Red light + emergency vehicle yield scenario:

Ego approaches a junction where its traffic light is frozen to RED.
A fire truck spawns behind ego with sirens on and drives toward it.
Ego must violate the red light to yield and let the fire truck pass.

XML parameters:
    distance          - how far behind ego to spawn fire truck (default 140)
    speed_increment   - fire truck speed above ego speed in km/h (default 25)
    trigger_distance  - distance at which fire truck triggers pressure (default 50)
    ev_idle_time      - max seconds to wait after fire truck approaches (default 15)
    ev_vehicle_type   - firetruck / ambulance / auto (default firetruck)
    ev_vehicle_model  - exact blueprint id (optional, highest priority)
    ev_vehicle_models - comma-separated blueprint ids/patterns (optional, highest priority)
    ev_spawn_min_distance - minimum rear spawn distance in meters (default 8)
    ev_spawn_max_distance - maximum rear spawn distance in meters (default no limit)
    ev_keep_driving  - if true, EV keeps driving until route end (default false)
    ev_target_speed_kmh - EV fixed target speed in km/h (default 50)
"""

import py_trees
import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (
    ActorTransformSetter,
    ActorDestroy,
    Idle,
    TrafficLightFreezer,
    AdaptiveConstantVelocityAgentBehavior,
    WaypointFollower,
)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import (
    Criterion,
    CollisionTest,
    YieldToEmergencyVehicleTest,
)
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (
    InTriggerDistanceToVehicle,
    WaitUntilInFront,
    DriveDistance,
)
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.scenariomanager.timer import GameTime
from srunner.scenariomanager.traffic_events import TrafficEvent, TrafficEventType
from srunner.tools.scenario_helper import get_closest_traffic_light
from agents.navigation.local_planner import RoadOption


class _EmergencyVehicleReachedEndCriterion(Criterion):
    """Emit a 100% route-completion event once EV reaches the configured endpoint."""

    def __init__(self, actor, goal_location, distance_threshold=6.0,
                 name="EmergencyVehicleReachedEnd"):
        super().__init__(name=name, actor=actor, optional=True, terminate_on_failure=False)
        self._goal_location = goal_location
        self._distance_threshold = float(distance_threshold)
        self._emitted = False

    def update(self):
        new_status = py_trees.common.Status.RUNNING
        if self._emitted or self.actor is None:
            return new_status

        location = CarlaDataProvider.get_location(self.actor)
        if location is None:
            return new_status

        if location.distance(self._goal_location) <= self._distance_threshold:
            self.test_status = "SUCCESS"
            event = TrafficEvent(event_type=TrafficEventType.ROUTE_COMPLETION, frame=GameTime.get_frame())
            event.set_message("Emergency vehicle reached endpoint; route marked complete at 100%.")
            event.set_dict({'route_completed': 100.0})
            self.events.append(event)
            self._emitted = True

        return new_status


class RedLightEmergencyYield(BasicScenario):
    """
    Ego's traffic light is frozen RED. A fire truck approaches from behind
    with sirens on. Ego must run the red light to yield.
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
    _EMERGENCY_KEYWORDS = ("firetruck", "ambulance", "rescue")
    _SPAWN_DISTANCE_OFFSETS = [0, 5, -5, 10, -10, 15, -15, 20, -20, 30, -30, 40]

    class _ZeroSpeedReference(object):
        """Reference actor shim to keep adaptive behavior at fixed speed."""
        @staticmethod
        def get_velocity():
            return carla.Vector3D(0.0, 0.0, 0.0)

    def __init__(self, world, ego_vehicles, config, debug_mode=False,
                 criteria_enable=True, timeout=180):
        self._world = world
        self._map = CarlaDataProvider.get_map()
        self.timeout = timeout

        p = config.other_parameters
        self._distance = float(p.get('distance', {}).get('value', 140))
        self._speed_increment = float(p.get('speed_increment', {}).get('value', 25))
        self._trigger_distance = float(p.get('trigger_distance', {}).get('value', 50))
        self._ev_idle_time = float(p.get('ev_idle_time', {}).get('value', 15))
        self._ev_vehicle_type = str(p.get('ev_vehicle_type', {}).get('value', 'firetruck')).strip().lower()
        self._ev_vehicle_model = str(p.get('ev_vehicle_model', {}).get('value', '')).strip()
        self._ev_vehicle_models = str(p.get('ev_vehicle_models', {}).get('value', '')).strip()
        self._ev_spawn_min_distance = float(p.get('ev_spawn_min_distance', {}).get('value', 8.0))
        self._ev_spawn_max_distance = float(p.get('ev_spawn_max_distance', {}).get('value', 1.0e9))
        self._ev_keep_driving = str(p.get('ev_keep_driving', {}).get('value', 'false')).strip().lower() in (
            "1", "true", "yes", "on"
        )
        self._ev_target_speed_kmh = float(p.get('ev_target_speed_kmh', {}).get('value', 50.0))
        self._end_distance = 50

        self._opt_dict = {
            'base_vehicle_threshold': 10,
            'detection_speed_ratio': 0.15,
            'use_bbs_detection': True,
            'base_min_distance': 1,
            'distance_ratio': 0.2,
            # Emergency vehicle should not stop at the forced-red junction lights.
            'ignore_traffic_lights': True,
            'ignore_stop_signs': True,
            # User requested emergency vehicle to not yield to leading vehicles.
            'ignore_vehicles': True,
        }

        self._trigger_location = config.trigger_points[0].location
        self._ego_wp = self._map.get_waypoint(self._trigger_location)
        self._ev_route_plan = None
        self._ev_end_location = None

        self._tl_dict = {}
        self._junction = None

        super().__init__("RedLightEmergencyYield",
                         ego_vehicles, config, world,
                         debug_mode, criteria_enable=criteria_enable)

    def _build_ev_route_plan(self, spawn_location):
        """
        Build an EV plan along the current route's waypoints, then extend it
        forward so EV keeps moving past the nominal route end.
        """
        route_data = getattr(self.config, "route", None) or []
        if not route_data:
            return None

        transforms = [elem[0] for elem in route_data if elem and elem[0] is not None]
        if not transforms:
            return None

        nearest_idx = min(
            range(len(transforms)),
            key=lambda i: transforms[i].location.distance(spawn_location)
        )
        sampled_transforms = transforms[nearest_idx::2]
        if transforms[-1].location.distance(sampled_transforms[-1].location) > 0.5:
            sampled_transforms.append(transforms[-1])

        plan = []
        last_location = None
        for tf in sampled_transforms:
            wp = self._map.get_waypoint(
                tf.location,
                project_to_road=True,
                lane_type=carla.LaneType.Driving
            )
            if wp is None:
                continue
            if last_location and wp.transform.location.distance(last_location) < 0.8:
                continue
            plan.append((wp, RoadOption.LANEFOLLOW))
            last_location = wp.transform.location

        if not plan:
            return None

        return plan

    @staticmethod
    def _split_csv(text):
        return [x.strip() for x in text.split(',') if x.strip()]

    def _get_ev_model_patterns(self):
        # Highest priority: explicit model list or single model from XML
        if self._ev_vehicle_models:
            return self._split_csv(self._ev_vehicle_models)
        if self._ev_vehicle_model:
            return [self._ev_vehicle_model]

        if self._ev_vehicle_type == 'ambulance':
            return self._AMBULANCE_PATTERNS + self._FIRETRUCK_PATTERNS
        if self._ev_vehicle_type == 'auto':
            return self._FIRETRUCK_PATTERNS + self._AMBULANCE_PATTERNS
        # Default and unknown values both prioritize firetruck behavior
        return self._FIRETRUCK_PATTERNS + self._AMBULANCE_PATTERNS

    def _resolve_ev_blueprint_ids(self):
        bp_library = self._world.get_blueprint_library()
        model_patterns = self._get_ev_model_patterns()

        resolved_ids = []
        seen_ids = set()
        for pattern in model_patterns:
            matched_ids = sorted({bp.id for bp in bp_library.filter(pattern)})
            for bp_id in matched_ids:
                if bp_id in seen_ids:
                    continue
                resolved_ids.append(bp_id)
                seen_ids.add(bp_id)

        if resolved_ids:
            return resolved_ids

        # Last resort: scan all vehicles for emergency-related keywords
        fallback_ids = []
        for bp in bp_library.filter('vehicle.*'):
            bp_id = bp.id
            lower_id = bp_id.lower()
            if any(k in lower_id for k in self._EMERGENCY_KEYWORDS) and bp_id not in seen_ids:
                fallback_ids.append(bp_id)
                seen_ids.add(bp_id)

        return sorted(fallback_ids)

    def _get_ev_start_transform_candidates(self, min_distance=None, max_distance=None):
        """Generate multiple rear spawn candidates to reduce spawn failures."""
        if min_distance is None:
            min_distance = self._ev_spawn_min_distance
        if max_distance is None:
            max_distance = self._ev_spawn_max_distance

        candidates = []
        seen = set()

        for offset in self._SPAWN_DISTANCE_OFFSETS:
            distance = max(8.0, self._distance + offset)
            if distance < min_distance or distance > max_distance:
                continue
            ev_points = self._ego_wp.previous(distance)
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
        # 1. Find junction ahead and freeze traffic lights
        self._find_junction_and_freeze_lights()

        # 2. Spawn emergency vehicle behind ego
        ev_blueprints = self._resolve_ev_blueprint_ids()
        if not ev_blueprints:
            raise Exception("No emergency vehicle blueprints matched current configuration")

        spawn_candidates = self._get_ev_start_transform_candidates()
        if not spawn_candidates:
            raise ValueError("Couldn't find any viable rear spawn waypoint for emergency vehicle")

        actor = None
        selected_bp = None
        selected_distance = None

        def _try_spawn(candidates):
            local_actor = None
            local_selected_bp = None
            local_selected_distance = None
            for bp_id in ev_blueprints:
                for distance, transform in candidates:
                    local_actor = CarlaDataProvider.request_new_actor(bp_id, transform)
                    if local_actor is not None:
                        self._ev_start_transform = transform
                        local_selected_bp = bp_id
                        local_selected_distance = distance
                        break
                if local_actor is not None:
                    break
            return local_actor, local_selected_bp, local_selected_distance

        actor, selected_bp, selected_distance = _try_spawn(spawn_candidates)

        # Fallback: if strict range failed, relax to avoid scenario being skipped.
        if actor is None and (self._ev_spawn_min_distance > 8.0 or self._ev_spawn_max_distance < 1.0e9):
            relaxed_candidates = self._get_ev_start_transform_candidates(min_distance=8.0, max_distance=1.0e9)
            used_keys = {
                (
                    round(tf.location.x, 2),
                    round(tf.location.y, 2),
                    round(tf.location.z, 2),
                    round(tf.rotation.yaw, 1),
                )
                for _, tf in spawn_candidates
            }
            relaxed_candidates = [
                (d, tf) for d, tf in relaxed_candidates
                if (
                    round(tf.location.x, 2),
                    round(tf.location.y, 2),
                    round(tf.location.z, 2),
                    round(tf.rotation.yaw, 1),
                ) not in used_keys
            ]
            if relaxed_candidates:
                print("[RedLightEmergencyYield] WARN: strict spawn range failed, retry with relaxed candidates")
                actor, selected_bp, selected_distance = _try_spawn(relaxed_candidates)

        if actor is None:
            tried_distances = ",".join(f"{d:.0f}" for d, _ in spawn_candidates[:8])
            raise Exception(
                "Couldn't spawn the emergency vehicle. "
                f"tried_models={ev_blueprints}, tried_distances={tried_distances}"
            )

        ev_dist = self._ev_start_transform.location.distance(self._trigger_location)
        print(f"[RedLightEmergencyYield] Using EV blueprint: {selected_bp}")
        print("[RedLightEmergencyYield] EV config: "
              f"requested_distance={self._distance:.1f}m, used_distance={selected_distance:.1f}m, "
              f"spawn_range=[{self._ev_spawn_min_distance:.1f},{self._ev_spawn_max_distance:.1f}]m, "
              f"fixed_target_speed={self._ev_target_speed_kmh:.1f}km/h, "
              f"ignore_vehicles={self._opt_dict['ignore_vehicles']}, "
              f"approx_spawn_to_trigger={ev_dist:.1f}m, "
              f"keep_driving={self._ev_keep_driving}")

        self._ev_route_plan = self._build_ev_route_plan(self._ev_start_transform.location)
        if self._ev_route_plan:
            self._ev_end_location = self._ev_route_plan[-1][0].transform.location
            print(f"[RedLightEmergencyYield] EV route plan points: {len(self._ev_route_plan)}")
        else:
            route_data = getattr(self.config, "route", None) or []
            if route_data:
                self._ev_end_location = route_data[-1][0].location
            print("[RedLightEmergencyYield] WARN: EV route plan unavailable; fallback to lane-follow mode")

        # Turn on emergency lights
        actor.set_light_state(carla.VehicleLightState(
            carla.VehicleLightState.Special1 | carla.VehicleLightState.Special2))

        self.other_actors.append(actor)

    def _find_junction_and_freeze_lights(self):
        """Walk forward from ego waypoint to find the junction,
        then set ego's traffic light to RED and freeze all lights."""
        self._tl_dict = {}
        wp = self._ego_wp
        junction_dist = 0
        while not wp.is_junction:
            nxt = wp.next(1.0)
            if not nxt:
                print("[RedLightEmergencyYield] WARNING: Failed to find junction ahead of trigger")
                break
            wp = nxt[0]
            junction_dist += 1
        if wp.is_junction:
            self._junction = wp.get_junction()
        else:
            self._junction = None

        tls = []
        if self._junction is not None:
            tls = list(self._world.get_traffic_lights_in_junction(self._junction.id))

        # Find ego's traffic light via landmark
        ego_tl = None
        if self._junction is not None:
            landmarks = self._ego_wp.get_landmarks_of_type(max(junction_dist + 20, 20), "1000001")
            if landmarks:
                ego_tl = self._world.get_traffic_light(landmarks[0])
            if ego_tl is None and tls:
                ego_tl = get_closest_traffic_light(self._ego_wp, tls)

        # UE5 fallback: junction api may return no traffic lights even when map has signals.
        if not tls:
            all_tls = list(self._world.get_actors().filter('traffic.traffic_light*'))
            if all_tls:
                if ego_tl is None:
                    ego_tl = get_closest_traffic_light(self._ego_wp, all_tls)
                nearby_tls = [
                    tl for tl in all_tls
                    if tl.get_location().distance(self._trigger_location) < 80.0
                ]
                tls = nearby_tls if nearby_tls else ([ego_tl] if ego_tl else [])
                print("[RedLightEmergencyYield] WARNING: No traffic lights from junction API, "
                      f"using fallback set (count={len(tls)})")

        if not tls:
            print("[RedLightEmergencyYield] WARNING: Could not find any traffic lights to freeze")
            return

        if ego_tl is None:
            ego_tl = get_closest_traffic_light(self._ego_wp, tls)

        # Always force RED in this scenario so ego's approach cannot turn green due to wrong TL matching.
        for tl in tls:
            self._tl_dict[tl] = carla.TrafficLightState.Red
        if ego_tl is not None and ego_tl not in self._tl_dict:
            self._tl_dict[ego_tl] = carla.TrafficLightState.Red
        mode_desc = "all RED (ego forced RED)"

        # Apply immediately so there is no visible green phase before behavior tree ticks.
        for tl, target_state in self._tl_dict.items():
            tl.set_state(target_state)
            tl.set_green_time(10000.0)
            tl.set_red_time(10000.0)
            tl.set_yellow_time(10000.0)

        print(f"  [RedLightEmergencyYield] Froze {len(self._tl_dict)} traffic lights ({mode_desc})")

    def _create_behavior(self):
        root = py_trees.composites.Sequence(name="RedLightEmergencyYield")

        # Phase 1: Freeze traffic lights (runs in parallel with everything)
        main = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE,
            name="MainBehavior")

        if self._tl_dict:
            main.add_child(TrafficLightFreezer(self._tl_dict))

        # Phase 2: Place fire truck at start position
        place_ev = py_trees.composites.Sequence(name="EVSequence")
        place_ev.add_child(ActorTransformSetter(
            self.other_actors[0], self._ev_start_transform))

        # Fire truck follows route waypoints with a fixed target speed.
        if self._ev_route_plan:
            ev_drive = WaypointFollower(
                self.other_actors[0],
                target_speed=self._ev_target_speed_kmh / 3.6,
                plan=self._ev_route_plan,
                avoid_collision=False,
                name="EVFollowRouteWaypoints",
            )
        else:
            ev_drive = AdaptiveConstantVelocityAgentBehavior(
                self.other_actors[0], self._ZeroSpeedReference(),
                speed_increment=self._ev_target_speed_kmh / 3.6,
                target_location=self._ev_end_location,
                opt_dict=self._opt_dict)

        if self._ev_keep_driving:
            # End scenario when EV reaches the end of its route plan.
            drive_phase = ev_drive
        else:
            # End condition 1: fire truck gets close, then wait idle time
            end1 = py_trees.composites.Sequence(name="EndCondition1")
            end1.add_child(InTriggerDistanceToVehicle(
                self.ego_vehicles[0], self.other_actors[0],
                self._trigger_distance))
            end1.add_child(Idle(self._ev_idle_time))

            # End condition 2: fire truck passes ego and drives away
            end2 = py_trees.composites.Sequence(name="EndCondition2")
            end2.add_child(WaitUntilInFront(
                self.other_actors[0], self.ego_vehicles[0]))
            end2.add_child(DriveDistance(
                self.other_actors[0], self._end_distance))

            drive_phase = py_trees.composites.Parallel(
                policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE,
                name="EVDriveAndWait")
            drive_phase.add_child(ev_drive)
            drive_phase.add_child(end1)
            drive_phase.add_child(end2)

        place_ev.add_child(drive_phase)
        main.add_child(place_ev)

        root.add_child(main)
        if not self._ev_keep_driving:
            root.add_child(ActorDestroy(self.other_actors[0]))
        return root

    def _create_test_criteria(self):
        criteria = [YieldToEmergencyVehicleTest(
            self.ego_vehicles[0], self.other_actors[0])]
        if self.route_mode and self._ev_end_location is not None:
            criteria.append(
                _EmergencyVehicleReachedEndCriterion(
                    self.other_actors[0],
                    self._ev_end_location,
                    distance_threshold=6.0,
                )
            )
        if not self.route_mode:
            criteria.append(CollisionTest(self.ego_vehicles[0]))
        return criteria

    def __del__(self):
        self.remove_all_actors()
