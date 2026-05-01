#!/usr/bin/env python

#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Front puddle scenario.

Spawns one or more static "puddle" props ahead of the trigger point.
The puddle model is configurable via XML and can be a custom imported prop.
"""

import carla

from srunner.scenarios.basic_scenario import BasicScenario
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import DriveDistance
from srunner.tools.scenario_helper import get_waypoint_in_distance


def _read_param(config, name, p_type, default):
    """Read a scalar parameter from config.other_parameters."""
    if hasattr(config, "other_parameters") and name in config.other_parameters:
        raw_value = config.other_parameters[name].get("value", None)
        if raw_value is None:
            return default
        try:
            return p_type(raw_value)
        except (TypeError, ValueError):
            print(f"WARNING: Invalid value for '{name}': '{raw_value}'. Using default '{default}'")
    return default


def _read_bool_param(config, name, default=False):
    """Read a boolean parameter from config.other_parameters."""
    value = _read_param(config, name, str, str(default))
    return str(value).strip().lower() in ("1", "true", "yes", "on")


class FrontPuddle(BasicScenario):
    """
    Place visual puddle props ahead of ego.

    Supported XML parameters (all optional):
      - distance: forward distance from trigger waypoint (m), default 20
      - lateral_offset: lateral shift wrt lane center (m), default 0
      - yaw_offset: extra yaw for spawned puddle (deg), default 0
      - puddle_model: blueprint id (e.g. static.prop.my_puddle), default static.prop.dirtdebris01
      - puddle_count: number of puddles, default 1
      - puddle_spacing: spacing between puddles along lane (m), default 3
      - post_distance: scenario completion distance after trigger (m), default 25
      - enable_friction: whether to add local friction triggers, default false
      - puddle_friction: trigger friction coefficient, default 0.6
      - friction_extent_x/y/z: trigger extents in meters, defaults 2.5 / 1.5 / 1.0
      - debug: draw debug points, default false
    """

    def __init__(self, world, ego_vehicles, config, randomize=False,
                 debug_mode=False, criteria_enable=True, timeout=60):
        self.timeout = timeout
        self._world = world
        self._map = CarlaDataProvider.get_map()
        self._randomize = randomize

        self._distance = _read_param(config, "distance", float, 20.0)
        self._lateral_offset = _read_param(config, "lateral_offset", float, 0.0)
        self._yaw_offset = _read_param(config, "yaw_offset", float, 0.0)
        self._puddle_model = _read_param(config, "puddle_model", str, "static.prop.dirtdebris01")
        self._puddle_count = max(1, _read_param(config, "puddle_count", int, 1))
        self._puddle_spacing = _read_param(config, "puddle_spacing", float, 3.0)
        self._post_distance = _read_param(config, "post_distance", float, 25.0)

        self._enable_friction = _read_bool_param(config, "enable_friction", False)
        self._puddle_friction = _read_param(config, "puddle_friction", float, 0.6)
        self._friction_extent_x = _read_param(config, "friction_extent_x", float, 2.5)
        self._friction_extent_y = _read_param(config, "friction_extent_y", float, 1.5)
        self._friction_extent_z = _read_param(config, "friction_extent_z", float, 1.0)

        self._debug = debug_mode or _read_bool_param(config, "debug", False)

        super().__init__("FrontPuddle", ego_vehicles, config, world, debug_mode, criteria_enable=criteria_enable)

    def _initialize_actors(self, config):
        """
        Spawn puddle props and optional local friction triggers ahead of the trigger point.
        """
        blueprint_library = self._world.get_blueprint_library()
        matched_blueprints = list(blueprint_library.filter(self._puddle_model))
        if not matched_blueprints:
            raise ValueError(
                f"Puddle model '{self._puddle_model}' not found in blueprint library. "
                "Please import/register the prop first."
            )

        reference_wp = self._map.get_waypoint(config.trigger_points[0].location)
        if reference_wp is None:
            raise ValueError("No valid reference waypoint for trigger point")

        visual_spawn_count = 0

        for idx in range(self._puddle_count):
            step_distance = self._distance + idx * self._puddle_spacing
            puddle_wp, _ = get_waypoint_in_distance(reference_wp, step_distance)
            if puddle_wp is None:
                raise ValueError(f"Could not compute waypoint at distance {step_distance:.2f}m")

            location = puddle_wp.transform.location
            if abs(self._lateral_offset) > 1e-6:
                right_vec = puddle_wp.transform.get_right_vector()
                location += carla.Location(
                    x=right_vec.x * self._lateral_offset,
                    y=right_vec.y * self._lateral_offset
                )

            if hasattr(self._world, "ground_projection"):
                ground_hit = self._world.ground_projection(location + carla.Location(z=1.0), 2.0)
                if ground_hit:
                    location = ground_hit.location

            rotation = puddle_wp.transform.rotation
            rotation.yaw += self._yaw_offset
            spawn_transform = carla.Transform(location, rotation)

            puddle_actor = self._spawn_puddle_actor_with_retry(spawn_transform)
            if puddle_actor is None:
                print(
                    f"WARNING: Failed to spawn puddle actor '{self._puddle_model}' at {spawn_transform.location}. "
                    "Keeping scenario alive and continuing."
                )
                if self._enable_friction:
                    self._spawn_friction_trigger(location)
                continue

            visual_spawn_count += 1
            puddle_actor.set_simulate_physics(False)
            self.other_actors.append(puddle_actor)

            if self._enable_friction:
                self._spawn_friction_trigger(location)

            if self._debug:
                self._world.debug.draw_point(
                    location + carla.Location(z=0.2),
                    size=0.18,
                    color=carla.Color(0, 80, 255),
                    life_time=60.0
                )

        if visual_spawn_count == 0:
            print(
                "WARNING: No visual puddle actor was spawned. "
                "This usually means the registered 'blueprint.*' actor is not runtime-spawnable "
                "in the current CARLA server build."
            )

    def _spawn_puddle_actor_with_retry(self, target_transform):
        """
        Spawn puddle actor robustly.

        Custom decal blueprints may fail when spawned exactly on the road surface.
        We retry with small XY offsets and lifted Z, then move the actor back.
        """
        offsets = [
            carla.Location(x=0.0, y=0.0, z=0.0),
            carla.Location(x=0.0, y=0.0, z=0.5),
            carla.Location(x=0.0, y=0.0, z=1.0),
            carla.Location(x=0.0, y=0.0, z=2.0),
            carla.Location(x=0.4, y=0.0, z=1.0),
            carla.Location(x=-0.4, y=0.0, z=1.0),
            carla.Location(x=0.0, y=0.4, z=1.0),
            carla.Location(x=0.0, y=-0.4, z=1.0),
        ]

        for offset in offsets:
            trial_location = carla.Location(
                x=target_transform.location.x + offset.x,
                y=target_transform.location.y + offset.y,
                z=target_transform.location.z + offset.z,
            )
            trial_transform = carla.Transform(trial_location, target_transform.rotation)
            actor = CarlaDataProvider.request_new_actor(
                self._puddle_model,
                trial_transform,
                rolename="prop",
                actor_category="misc",
            )
            if actor is not None:
                if abs(offset.x) > 1e-6 or abs(offset.y) > 1e-6 or abs(offset.z) > 1e-6:
                    actor.set_transform(target_transform)
                return actor

        return None

    def _spawn_friction_trigger(self, location):
        """Spawn a local friction trigger near a puddle to emulate slippery road."""
        try:
            friction_bp = self._world.get_blueprint_library().find("static.trigger.friction")
        except RuntimeError:
            print("WARNING: 'static.trigger.friction' blueprint not available, skipping friction trigger")
            return

        friction_bp.set_attribute("friction", str(self._puddle_friction))
        friction_bp.set_attribute("extent_x", str(self._friction_extent_x))
        friction_bp.set_attribute("extent_y", str(self._friction_extent_y))
        friction_bp.set_attribute("extent_z", str(self._friction_extent_z))

        trigger_transform = carla.Transform(location + carla.Location(z=0.2))
        trigger_actor = self._world.try_spawn_actor(friction_bp, trigger_transform)
        if trigger_actor is None:
            print(f"WARNING: Failed to spawn friction trigger at {trigger_transform.location}")
            return

        self.other_actors.append(trigger_actor)

    def _create_behavior(self):
        """
        Keep scenario active until ego has travelled enough distance after activation.
        """
        completion_distance = max(10.0, self._distance + self._post_distance)
        return DriveDistance(self.ego_vehicles[0], completion_distance)

    def _create_test_criteria(self):
        if self.route_mode:
            return []
        return [CollisionTest(self.ego_vehicles[0])]

    def __del__(self):
        self.remove_all_actors()
