#!/usr/bin/env python

"""
CenterBarrierObstacle scenario for UE5 maps.

Places one or more static obstacles at the center of the ego lane.
Use obstacle_types to mix normal barrier and weird barrier props.
"""

import py_trees
import carla
import random
import math

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import Idle
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import DriveDistance
from srunner.scenarios.basic_scenario import BasicScenario


class CenterBarrierObstacle(BasicScenario):
    """
    Put one or more static props in road center ahead of trigger.

    XML Parameters:
      forward_distance: distance from trigger to first obstacle (default 15)
      lateral_offset: lateral offset from lane center in meters (default 0)
      lateral_jitter: random lateral offset range in meters for each obstacle
        (default 0.8, means [-0.8, 0.8] around lateral_offset)
      num_obstacles: number of obstacles to place (default 1)
      spacing: longitudinal spacing between obstacles (default 7)
      asset_type: preferred odd asset type (single), e.g. vendingmachine / balloon / foam_box
      asset_types: comma-separated odd asset types (higher priority than obstacle_type/obstacle_types)
      obstacle_type: single prop suffix/full id (default streetbarrier)
      obstacle_types: comma-separated list, e.g. streetbarrier,warningaccident,trafficwarning
      lay_down: whether to lay the asset down on the ground (default false)
      lay_down_angle: lay-down angle in degrees (default 90)
      lay_down_axis: roll or pitch (default roll)
      z_offset: added z for obstacle placement (default 0.2)
      yaw_offset: added yaw in degrees (default 0)
      yaw_jitter: random yaw range in degrees for each obstacle (default 20, means [-20, 20])
    """

    def __init__(self, world, ego_vehicles, config, randomize=False,
                 debug_mode=False, criteria_enable=True, timeout=180):
        self._world = world
        self._map = CarlaDataProvider.get_map()
        self._rng = CarlaDataProvider.get_random_seed() or random.Random()
        self.timeout = timeout

        self._trigger_location = config.trigger_points[0].location

        p = config.other_parameters if hasattr(config, 'other_parameters') else {}
        self._forward_distance = float(p.get('forward_distance', {}).get('value', 15.0))
        self._lateral_offset = float(p.get('lateral_offset', {}).get('value', 0.0))
        self._lateral_jitter = abs(float(p.get('lateral_jitter', {}).get('value', 0.8)))
        self._num_obstacles = max(1, int(p.get('num_obstacles', {}).get('value', 1)))
        self._spacing = float(p.get('spacing', {}).get('value', 7.0))
        self._yaw_offset = float(p.get('yaw_offset', {}).get('value', 0.0))
        self._yaw_jitter = abs(float(p.get('yaw_jitter', {}).get('value', 20.0)))
        self._lay_down = str(p.get('lay_down', {}).get('value', 'false')).strip().lower() in (
            '1', 'true', 'yes', 'on'
        )
        self._lay_down_angle = float(p.get('lay_down_angle', {}).get('value', 90.0))
        self._lay_down_axis = str(p.get('lay_down_axis', {}).get('value', 'roll')).strip().lower()
        if self._lay_down_axis not in ('roll', 'pitch'):
            self._lay_down_axis = 'roll'
        # For laid-down assets, default z_offset to 0.0 so it is easier to contact ground.
        default_z_offset = 0.0 if self._lay_down else 0.2
        self._z_offset = float(p.get('z_offset', {}).get('value', default_z_offset))

        # Priority: asset_types > asset_type > obstacle_types > obstacle_type
        asset_types_raw = p.get('asset_types', {}).get('value', '')
        asset_type_raw = p.get('asset_type', {}).get('value', '').strip()
        obstacle_types_raw = p.get('obstacle_types', {}).get('value', '')
        obstacle_type_raw = p.get('obstacle_type', {}).get('value', 'streetbarrier').strip()

        if asset_types_raw.strip():
            self._obstacle_types = [s.strip() for s in asset_types_raw.split(',') if s.strip()]
        elif asset_type_raw:
            self._obstacle_types = [asset_type_raw]
        elif obstacle_types_raw.strip():
            self._obstacle_types = [s.strip() for s in obstacle_types_raw.split(',') if s.strip()]
        else:
            self._obstacle_types = [obstacle_type_raw]

        if not self._obstacle_types:
            self._obstacle_types = ['streetbarrier']

        self._ego_drive_distance = max(35.0, self._forward_distance + self._num_obstacles * self._spacing + 20.0)

        super().__init__(
            "CenterBarrierObstacle",
            ego_vehicles,
            config,
            world,
            debug_mode,
            criteria_enable=criteria_enable,
        )

    @staticmethod
    def _normalize_type_name(type_name):
        name = (type_name or '').strip()
        if not name:
            return 'static.prop.streetbarrier'
        if '.' not in name:
            return 'static.prop.{}'.format(name)
        return name

    @staticmethod
    def _advance_waypoint(wp, distance):
        """Advance waypoint by distance using small fixed steps for robustness."""
        if wp is None or distance <= 0.0:
            return wp

        remaining = float(distance)
        current = wp
        while remaining > 0.01:
            step = min(2.0, remaining)
            nxt = current.next(step)
            if not nxt:
                break
            current = nxt[0]
            remaining -= step
        return current

    @staticmethod
    def _is_bad_spawn(actual_loc, expected_loc):
        """Detect UE5 occasional invalid prop placement (e.g. snapped to world origin)."""
        vals = [actual_loc.x, actual_loc.y, actual_loc.z, expected_loc.x, expected_loc.y, expected_loc.z]
        if any(not math.isfinite(v) for v in vals):
            return True

        # Typical bad case observed in UE5: actor reports near (0, 0, 0) unexpectedly.
        if abs(actual_loc.x) < 0.5 and abs(actual_loc.y) < 0.5 and (
            abs(expected_loc.x) > 5.0 or abs(expected_loc.y) > 5.0
        ):
            return True

        dx = actual_loc.x - expected_loc.x
        dy = actual_loc.y - expected_loc.y
        dz = actual_loc.z - expected_loc.z
        return (dx * dx + dy * dy + dz * dz) > (25.0 * 25.0)

    def _resolve_obstacle_blueprint(self, requested_type):
        bp_lib = self._world.get_blueprint_library()

        requested_key = (requested_type or '').strip().lower()
        odd_aliases = {
            # Friendly aliases for odd assets.
            'balloon': ['warningaccident', 'trafficwarning'],
            'foam_box': ['dumpster02', 'dumpster'],
            'vending': ['vendingmachine'],
            'kiosk': ['kiosk_01'],
            'plantpot': ['plantpot02'],
        }

        mapped = odd_aliases.get(requested_key, [requested_type])
        requested_candidates = [self._normalize_type_name(x) for x in mapped]

        candidates = requested_candidates + [
            # Weird assets first, then common road obstacles.
            'static.prop.vendingmachine',
            'static.prop.kiosk_01',
            'static.prop.plantpot02',
            'static.prop.dumpster02',
            'static.prop.dumpster',
            'static.prop.warningaccident',
            'static.prop.trafficwarning',
            'static.prop.streetbarrier',
            'static.prop.warningconstruction',
            'static.prop.trafficcone02',
        ]

        visited = set()
        for bp_name in candidates:
            if bp_name in visited:
                continue
            visited.add(bp_name)
            bp_list = list(bp_lib.filter(bp_name))
            if bp_list:
                return bp_list[0], bp_name

        return None, None

    def _initialize_actors(self, config):
        print("\n{}".format('=' * 60))
        print("  SCENARIO LOADED: CenterBarrierObstacle")
        print("  Trigger location: {}".format(self._trigger_location))
        print("{}\n".format('=' * 60))

        ref_wp = self._map.get_waypoint(self._trigger_location)
        if ref_wp is None:
            print("  [WARN] Could not find trigger waypoint, skip obstacle spawn")
            return

        current_wp = self._advance_waypoint(ref_wp, self._forward_distance)
        spawned = 0

        for idx in range(self._num_obstacles):
            req_type = self._obstacle_types[idx % len(self._obstacle_types)]
            blueprint, selected_name = self._resolve_obstacle_blueprint(req_type)
            if blueprint is None:
                print("  [WARN] No obstacle blueprint found for '{}', skipping".format(req_type))
                continue

            center = current_wp.transform.location
            right = current_wp.transform.get_right_vector()
            random_lateral = (
                self._rng.uniform(-self._lateral_jitter, self._lateral_jitter)
                if self._lateral_jitter > 0.0 else 0.0
            )
            lateral = self._lateral_offset + random_lateral
            loc = carla.Location(
                x=center.x + lateral * right.x,
                y=center.y + lateral * right.y,
                z=center.z + self._z_offset,
            )
            pitch = 0.0
            roll = 0.0
            if self._lay_down:
                if self._lay_down_axis == 'pitch':
                    pitch += self._lay_down_angle
                else:
                    roll += self._lay_down_angle
            random_yaw = self._rng.uniform(-self._yaw_jitter, self._yaw_jitter) if self._yaw_jitter > 0.0 else 0.0
            rot = carla.Rotation(
                pitch=pitch,
                yaw=current_wp.transform.rotation.yaw + self._yaw_offset + random_yaw,
                roll=roll,
            )

            actor = self._world.try_spawn_actor(blueprint, carla.Transform(loc, rot))
            if actor is None:
                loc.z += 0.8
                actor = self._world.try_spawn_actor(blueprint, carla.Transform(loc, rot))

            # Guard against invalid UE5 spawn transforms for some static props.
            if actor is not None:
                try:
                    actual_loc = actor.get_transform().location
                    bad_spawn = self._is_bad_spawn(actual_loc, loc)
                except RuntimeError:
                    bad_spawn = True

                if bad_spawn:
                    print(
                        "  [WARN] Obstacle #{} spawned at invalid transform; fallback to streetbarrier".format(
                            idx + 1
                        )
                    )
                    try:
                        actor.destroy()
                    except RuntimeError:
                        pass
                    actor = None

                    fallback_bp, fallback_name = self._resolve_obstacle_blueprint('streetbarrier')
                    if fallback_bp is not None:
                        safe_rot = carla.Rotation(
                            pitch=0.0,
                            yaw=current_wp.transform.rotation.yaw,
                            roll=0.0,
                        )
                        for dz in (self._z_offset, self._z_offset + 0.8):
                            safe_loc = carla.Location(
                                x=center.x + self._lateral_offset * right.x,
                                y=center.y + self._lateral_offset * right.y,
                                z=center.z + dz,
                            )
                            actor = self._world.try_spawn_actor(
                                fallback_bp, carla.Transform(safe_loc, safe_rot)
                            )
                            if actor is not None:
                                selected_name = fallback_name
                                break

            if actor is None:
                print("  [WARN] Failed to spawn obstacle #{} ({})".format(idx + 1, selected_name))
            else:
                self.other_actors.append(actor)
                CarlaDataProvider._carla_actor_pool[actor.id] = actor
                spawned += 1
                print(
                    "  [INFO] Obstacle #{} spawned ({}) at ({:.2f}, {:.2f}, {:.2f}), yaw={:.1f} (yaw_jitter={:+.1f}, lateral_jitter={:+.2f})".format(
                        idx + 1,
                        selected_name,
                        actor.get_transform().location.x,
                        actor.get_transform().location.y,
                        actor.get_transform().location.z,
                        actor.get_transform().rotation.yaw,
                        random_yaw,
                        random_lateral,
                    )
                )

            nxt = current_wp.next(max(0.5, self._spacing))
            if nxt:
                current_wp = nxt[0]

        print("  [INFO] Spawned {}/{} center obstacles".format(spawned, self._num_obstacles))

    def _create_behavior(self):
        behavior = py_trees.composites.Sequence(name="CenterBarrierObstacle")

        end_condition = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE,
            name="CenterBarrierObstacleEnd",
        )
        end_condition.add_child(DriveDistance(self.ego_vehicles[0], self._ego_drive_distance))
        end_condition.add_child(Idle(self.timeout))

        behavior.add_child(end_condition)
        return behavior

    def _create_test_criteria(self):
        if self.route_mode:
            return []
        return [CollisionTest(self.ego_vehicles[0])]

    def __del__(self):
        self.remove_all_actors()
