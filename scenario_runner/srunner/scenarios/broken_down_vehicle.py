#!/usr/bin/env python
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Broken Down Vehicle scenario for UE5 maps.
A vehicle is stopped in the middle of the road with hazard lights on.
The ego vehicle must detect and avoid it.
"""

from __future__ import print_function

import py_trees
import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import Idle
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (
    InTriggerDistanceToLocation, DriveDistance)
from srunner.scenarios.basic_scenario import BasicScenario


class VehicleLightsBehavior(py_trees.behaviour.Behaviour):
    """Keeps the requested light state on a vehicle while the behavior is running."""

    def __init__(self, vehicle, light_state, name="VehicleLights"):
        super().__init__(name)
        self._vehicle = vehicle
        self._light_state = light_state

    def update(self):
        try:
            self._vehicle.set_light_state(self._light_state)
        except RuntimeError:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        pass


class BrokenDownVehicle(BasicScenario):
    """
    A vehicle is broken down and stopped in the road with hazard lights on.
    The ego vehicle must detect and avoid it.

    XML Parameters:
        forward_distance: How far ahead of trigger point to place the vehicle (default 15)
        lateral_offset: Lateral offset from road center (default 0)
        vehicle_filter: Blueprint filter for the broken vehicle (default 'vehicle.*')
        yaw_offset: Extra yaw rotation in degrees relative to road direction (default 0)
        hazard_lights: Whether to enable vehicle hazard lights (default true)
        spawn_warning_props: Whether to spawn warning barriers/cones near the broken vehicle (default false)
        warning_prop_type: Prop blueprint suffix or full id, e.g. 'streetbarrier' or 'static.prop.streetbarrier'
        warning_prop_count: Number of warning props to place (default 3)
        warning_prop_front_distance: Distance in front of broken vehicle for warning props (optional)
        warning_prop_rear_distance: Distance behind broken vehicle for warning props (optional)
        warning_prop_longitudinal: Fallback distance behind broken vehicle for warning props (default 4.0)
        warning_prop_lateral_spacing: Lateral spacing for warning props (default 1.2)
        warning_prop_yaw_offset: Extra yaw rotation for warning props in degrees (default 0)
        warning_prop_z_offset: Extra z offset for warning props (default 0.2)
    """

    def __init__(self, world, ego_vehicles, config, debug_mode=False,
                 criteria_enable=True, timeout=60):
        self._wmap = CarlaDataProvider.get_map()
        self._trigger_location = config.trigger_points[0].location
        self._reference_waypoint = self._wmap.get_waypoint(self._trigger_location)
        self._rng = CarlaDataProvider.get_random_seed()

        self._min_trigger_dist = 30.0
        self._ego_end_distance = 40
        self.timeout = timeout

        # Read parameters from XML
        self._forward_distance = 15.0
        self._lateral_offset = 0.0
        self._vehicle_filter = 'vehicle.*'
        self._yaw_offset = 0.0
        self._hazard_lights = True
        self._spawn_warning_props = False
        self._warning_prop_type = 'streetbarrier'
        self._warning_prop_count = 3
        self._warning_prop_front_distance = None
        self._warning_prop_rear_distance = None
        self._warning_prop_longitudinal = 4.0
        self._warning_prop_lateral_spacing = 1.2
        self._warning_prop_yaw_offset = 0.0
        self._warning_prop_z_offset = 0.2
        if hasattr(config, 'other_parameters'):
            if 'forward_distance' in config.other_parameters:
                self._forward_distance = float(
                    config.other_parameters['forward_distance'].get('value', '15'))
            if 'lateral_offset' in config.other_parameters:
                self._lateral_offset = float(
                    config.other_parameters['lateral_offset'].get('value', '0'))
            if 'vehicle_filter' in config.other_parameters:
                self._vehicle_filter = config.other_parameters['vehicle_filter'].get(
                    'value', 'vehicle.*')
            if 'yaw_offset' in config.other_parameters:
                self._yaw_offset = float(
                    config.other_parameters['yaw_offset'].get('value', '0'))
            if 'hazard_lights' in config.other_parameters:
                self._hazard_lights = str(
                    config.other_parameters['hazard_lights'].get('value', 'true')
                ).strip().lower() in ('1', 'true', 'yes', 'on')
            if 'spawn_warning_props' in config.other_parameters:
                self._spawn_warning_props = str(
                    config.other_parameters['spawn_warning_props'].get('value', 'false')
                ).strip().lower() in ('1', 'true', 'yes', 'on')
            if 'warning_prop_type' in config.other_parameters:
                self._warning_prop_type = config.other_parameters['warning_prop_type'].get(
                    'value', 'streetbarrier')
            if 'warning_prop_count' in config.other_parameters:
                self._warning_prop_count = max(
                    1, int(config.other_parameters['warning_prop_count'].get('value', '3')))
            if 'warning_prop_front_distance' in config.other_parameters:
                self._warning_prop_front_distance = abs(float(
                    config.other_parameters['warning_prop_front_distance'].get('value', '10.0')))
            if 'warning_prop_rear_distance' in config.other_parameters:
                self._warning_prop_rear_distance = abs(float(
                    config.other_parameters['warning_prop_rear_distance'].get('value', '10.0')))
            if 'warning_prop_longitudinal' in config.other_parameters:
                self._warning_prop_longitudinal = float(
                    config.other_parameters['warning_prop_longitudinal'].get('value', '4.0'))
            if 'warning_prop_lateral_spacing' in config.other_parameters:
                self._warning_prop_lateral_spacing = float(
                    config.other_parameters['warning_prop_lateral_spacing'].get('value', '1.2'))
            if 'warning_prop_yaw_offset' in config.other_parameters:
                self._warning_prop_yaw_offset = float(
                    config.other_parameters['warning_prop_yaw_offset'].get('value', '0.0'))
            if 'warning_prop_z_offset' in config.other_parameters:
                self._warning_prop_z_offset = float(
                    config.other_parameters['warning_prop_z_offset'].get('value', '0.2'))

        if self._hazard_lights:
            self._vehicle_light_state = carla.VehicleLightState.All
        else:
            self._vehicle_light_state = carla.VehicleLightState.NONE

        super().__init__("BrokenDownVehicle", ego_vehicles, config, world,
                         debug_mode, criteria_enable=criteria_enable)

    def _spawn_warning_props_around_vehicle(self, broken_vehicle, reference_transform=None):
        """Spawn warning props around the broken-down vehicle."""
        blueprint_library = self.world.get_blueprint_library()

        configured_type = self._warning_prop_type
        if '.' not in configured_type:
            configured_type = "static.prop.{}".format(configured_type)

        candidate_ids = [
            configured_type,
            'static.prop.trafficwarning',
            'static.prop.warningconstruction',
            'static.prop.streetbarrier',
            'static.prop.trafficcone02',
        ]

        selected_bp = None
        selected_name = None
        for bp_id in candidate_ids:
            bp_list = list(blueprint_library.filter(bp_id))
            if bp_list:
                selected_bp = bp_list[0]
                selected_name = bp_id
                break

        if selected_bp is None:
            print("  [WARN] No warning prop blueprint found, skipping warning props")
            return

        vehicle_tf = reference_transform if reference_transform is not None else broken_vehicle.get_transform()
        base_loc = vehicle_tf.location
        forward = vehicle_tf.get_forward_vector()
        right = vehicle_tf.get_right_vector()

        lat = self._warning_prop_lateral_spacing
        offsets = []

        # Preferred layout: place warning props at front and rear distances.
        if self._warning_prop_front_distance is not None or self._warning_prop_rear_distance is not None:
            front_d = self._warning_prop_front_distance
            if front_d is None:
                front_d = abs(self._warning_prop_longitudinal)
            rear_d = self._warning_prop_rear_distance
            if rear_d is None:
                rear_d = abs(self._warning_prop_longitudinal)

            offsets = [
                (front_d, 0.0),    # front of broken vehicle
                (-rear_d, 0.0),    # rear of broken vehicle
            ]

            # Optional extra markers near front/rear points for better visibility.
            if self._warning_prop_count > 2:
                extras = [
                    (front_d, -lat),
                    (front_d, lat),
                    (-rear_d, -lat),
                    (-rear_d, lat),
                ]
                offsets.extend(extras[:self._warning_prop_count - 2])
        else:
            rear_d = self._warning_prop_longitudinal
            pattern_offsets = [
                (-rear_d, 0.0),
                (-(rear_d + 1.2), -lat),
                (-(rear_d + 1.2), lat),
                (-1.8, -(lat + 0.6)),
                (-1.8, lat + 0.6),
            ]
            offsets = pattern_offsets[:self._warning_prop_count]

        spawned = 0
        for idx, (longitudinal, lateral) in enumerate(offsets, start=1):
            raw_x = base_loc.x + forward.x * longitudinal + right.x * lateral
            raw_y = base_loc.y + forward.y * longitudinal + right.y * lateral
            probe_loc = carla.Location(x=raw_x, y=raw_y, z=base_loc.z)

            # Snap to road height so props contact ground instead of floating.
            ground_z = base_loc.z
            ground_wp = self._wmap.get_waypoint(probe_loc)
            if ground_wp is not None:
                ground_z = ground_wp.transform.location.z
            elif self._reference_waypoint is not None:
                ground_z = self._reference_waypoint.transform.location.z

            loc = carla.Location(
                x=raw_x,
                y=raw_y,
                z=ground_z + self._warning_prop_z_offset
            )
            rot = carla.Rotation(
                pitch=0.0,
                yaw=vehicle_tf.rotation.yaw + 180.0 + self._warning_prop_yaw_offset,
                roll=0.0
            )
            actor = self.world.try_spawn_actor(selected_bp, carla.Transform(loc, rot))
            if actor is None:
                # Small fallback lift for rare spawn failures on exact ground contact.
                loc.z += 0.08
                actor = self.world.try_spawn_actor(selected_bp, carla.Transform(loc, rot))
            if actor:
                CarlaDataProvider._carla_actor_pool[actor.id] = actor
                self.other_actors.append(actor)
                spawned += 1
                print(
                    "  [INFO] Warning prop #{} at ({:.2f}, {:.2f}, {:.2f}), yaw={:.1f}".format(
                        idx, loc.x, loc.y, loc.z, rot.yaw
                    )
                )
            else:
                print("  [WARN] Failed to spawn warning prop #{}".format(idx))

        print("  [INFO] Spawned {}/{} warning props ({})".format(
            spawned, len(offsets), selected_name))

    def _initialize_actors(self, config):
        print(f"\n{'='*60}")
        print(f"  SCENARIO LOADED: BrokenDownVehicle")
        print(f"  Trigger location: {self._trigger_location}")
        print(f"{'='*60}\n")

        # Calculate spawn position: forward from trigger point
        forward_vec = self._reference_waypoint.transform.get_forward_vector()
        right_vec = self._reference_waypoint.transform.get_right_vector()

        spawn_loc = self._reference_waypoint.transform.location + carla.Location(
            self._forward_distance * forward_vec.x + self._lateral_offset * right_vec.x,
            self._forward_distance * forward_vec.y + self._lateral_offset * right_vec.y,
            0
        )

        # Snap to road
        spawn_wp = self._wmap.get_waypoint(spawn_loc)
        if spawn_wp:
            spawn_transform = spawn_wp.transform
            spawn_transform.location.z += 0.5
        else:
            spawn_transform = carla.Transform(
                spawn_loc,
                self._reference_waypoint.transform.rotation
            )

        # Apply yaw offset so the vehicle can be angled / sideways
        if self._yaw_offset != 0.0:
            spawn_transform.rotation.yaw += self._yaw_offset

        print(f"  [INFO] Spawning broken vehicle at {spawn_transform.location}")

        # Pick a vehicle blueprint using the configured filter
        blueprint_library = self.world.get_blueprint_library()
        vehicle_bps = list(blueprint_library.filter(self._vehicle_filter))
        # If using default filter, exclude large vehicles
        if self._vehicle_filter == 'vehicle.*':
            vehicle_bps = [bp for bp in vehicle_bps
                           if 'firetruck' not in bp.id and 'ambulance' not in bp.id
                           and 'sprinter' not in bp.id and 'fuso' not in bp.id]

        if not vehicle_bps:
            if self._vehicle_filter != 'vehicle.*':
                print(f"  [WARN] No blueprints for filter '{self._vehicle_filter}', falling back to any large vehicle")
                vehicle_bps = [bp for bp in blueprint_library.filter('vehicle.*')
                               if 'truck' in bp.id or 'bus' in bp.id or 'carlacola' in bp.id]
            if not vehicle_bps:
                print(f"  [ERROR] No vehicle blueprints available")
                return

        vehicle_bp = self._rng.choice(vehicle_bps)

        # Spawn the broken-down vehicle
        vehicle = self.world.try_spawn_actor(vehicle_bp, spawn_transform)
        if vehicle is None:
            spawn_transform.location.z += 1.0
            vehicle = self.world.try_spawn_actor(vehicle_bp, spawn_transform)

        if vehicle is None:
            print(f"  [ERROR] Could not spawn broken vehicle")
            return

        # Apply handbrake to keep it stationary
        control = carla.VehicleControl()
        control.hand_brake = True
        vehicle.apply_control(control)

        vehicle.set_light_state(self._vehicle_light_state)

        CarlaDataProvider._carla_actor_pool[vehicle.id] = vehicle
        self.other_actors.append(vehicle)
        self._broken_vehicle = vehicle

        print(f"  [INFO] Broken vehicle spawned: {vehicle_bp.id}")
        if self._spawn_warning_props:
            # Use the intended spawn transform directly so prop placement is stable
            # even before physics ticks update actor transforms.
            self._spawn_warning_props_around_vehicle(vehicle, reference_transform=spawn_transform)

    def _create_behavior(self):
        sequence = py_trees.composites.Sequence(name="BrokenDownVehicle")

        collision_location = self._reference_waypoint.transform.location

        # Trigger when ego is close
        trigger = InTriggerDistanceToLocation(
            self.ego_vehicles[0], collision_location, self._min_trigger_dist)
        sequence.add_child(trigger)

        # Keep the configured vehicle lights forced while ego drives past.
        if hasattr(self, '_broken_vehicle'):
            end_condition = py_trees.composites.Parallel(
                policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE,
                name="EndCondition")
            end_condition.add_child(VehicleLightsBehavior(
                self._broken_vehicle,
                self._vehicle_light_state,
                name="BrokenVehicleLights"))
            end_condition.add_child(DriveDistance(
                self.ego_vehicles[0], self._ego_end_distance, name="EgoDriveDistance"))
            sequence.add_child(end_condition)
        else:
            sequence.add_child(DriveDistance(
                self.ego_vehicles[0], self._ego_end_distance, name="EndCondition"))

        return sequence

    def _create_test_criteria(self):
        if self.route_mode:
            return []
        return [CollisionTest(self.ego_vehicles[0])]

    def __del__(self):
        self.remove_all_actors()
