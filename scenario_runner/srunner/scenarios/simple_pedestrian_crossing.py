#!/usr/bin/env python
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Simple Pedestrian Crossing scenario for UE5 maps.
Simplified version that doesn't require junctions or sidewalks.
Uses WalkerAIController for proper pedestrian movement.
"""

from __future__ import print_function

import py_trees
import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import Idle
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (InTriggerDistanceToLocation,
                                                                               DriveDistance)
from srunner.scenarios.basic_scenario import BasicScenario


class ScenarioNotification(py_trees.behaviour.Behaviour):
    """Simple notification behavior that prints scenario trigger info"""
    def __init__(self, message, name="ScenarioNotification"):
        super().__init__(name)
        self._message = message
        self._triggered = False

    def update(self):
        if not self._triggered:
            print(f"\n{'='*60}")
            print(f"  {self._message}")
            print(f"{'='*60}\n")
            self._triggered = True
        return py_trees.common.Status.SUCCESS


class WalkerCrossingBehavior(py_trees.behaviour.Behaviour):
    """
    Behavior that controls an existing walker using set_transform.
    The walker crosses the road to the destination location.
    Uses direct transform manipulation for reliable movement in UE5.
    """

    def __init__(self, walker, destination, speed=1.5, name="WalkerCrossing"):
        super().__init__(name)
        self._walker = walker
        self._destination = destination
        self._speed = speed
        self._initialized = False
        self._sink_dist = 2.0
        self._direction = None
        self._target_yaw = None

    def initialise(self):
        """Calculate direction and prepare for walking"""
        if self._initialized:
            return

        # Check if walker still exists
        if self._walker is None:
            print(f"  [WARNING] Walker is None, skipping control setup")
            return

        try:
            # Check if walker is still alive
            loc = self._walker.get_location()
            if loc is None:
                print(f"  [WARNING] Walker location is None, may be destroyed")
                return

            # Calculate yaw angle to destination
            import math
            dx = self._destination.x - loc.x
            dy = self._destination.y - loc.y
            self._target_yaw = math.degrees(math.atan2(dy, dx))

            # Calculate direction vector
            length = (dx**2 + dy**2)**0.5
            if length > 0.1:
                self._direction = carla.Vector3D(dx/length, dy/length, 0)
            else:
                self._direction = carla.Vector3D(1, 0, 0)

            # Set walker's initial rotation to face destination
            current_transform = self._walker.get_transform()
            current_transform.rotation.yaw = self._target_yaw
            self._walker.set_transform(current_transform)

            # Also try WalkerControl (may work in some CARLA versions)
            control = carla.WalkerControl()
            control.speed = self._speed
            rotation = carla.Rotation(pitch=0, yaw=self._target_yaw, roll=0)
            control.direction = rotation.get_forward_vector()
            control.jump = False
            self._walker.apply_control(control)

            self._initialized = True
            print(f"  [INFO] Walker ready, speed={self._speed}, yaw={self._target_yaw:.1f}")

        except RuntimeError as e:
            print(f"  [WARNING] Walker control failed: {e}")
            return

    def update(self):
        """Move walker by setting transform + apply_control for animation"""
        if not self._initialized:
            return py_trees.common.Status.SUCCESS

        try:
            current_transform = self._walker.get_transform()
            loc = current_transform.location
            if loc is None:
                return py_trees.common.Status.SUCCESS

            if loc.distance(self._destination) < self._sink_dist:
                # Stop animation
                control = carla.WalkerControl()
                control.speed = 0
                self._walker.apply_control(control)
                print(f"  [INFO] Walker reached destination")
                return py_trees.common.Status.SUCCESS

            # Apply control to trigger walk animation
            control = carla.WalkerControl()
            control.speed = self._speed
            control.direction = carla.Rotation(pitch=0, yaw=self._target_yaw, roll=0).get_forward_vector()
            self._walker.apply_control(control)

            # Move walker by set_transform for reliable displacement
            delta_time = 0.05
            move_distance = self._speed * delta_time
            new_loc = carla.Location(
                loc.x + self._direction.x * move_distance,
                loc.y + self._direction.y * move_distance,
                loc.z
            )
            new_transform = carla.Transform(new_loc, carla.Rotation(pitch=0, yaw=self._target_yaw, roll=0))
            self._walker.set_transform(new_transform)

        except RuntimeError:
            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        """Nothing to clean up"""
        pass


class SimplePedestrianCrossing(BasicScenario):
    """
    Simplified pedestrian crossing scenario for UE5 maps.
    Spawns pedestrians directly at the trigger point without requiring
    junctions or sidewalks.

    XML Parameters:
        blocker_type: Type of view blocker prop name
            - "none": No blocker
            - "advertisement": Advertisement board (default)
            - "busstop": Bus stop shelter
            - "vendingmachine": Vending machine
            - "container": Large container
            - Or any other prop suffix / full static.prop.* blueprint id
        pedestrian_count: Number of walkers to spawn (1-3).
            If omitted, it is randomized to 1-3.
    """

    def __init__(self, world, ego_vehicles, config, debug_mode=False, criteria_enable=True, timeout=60):
        self._wmap = CarlaDataProvider.get_map()
        self._trigger_location = config.trigger_points[0].location
        self._reference_waypoint = self._wmap.get_waypoint(self._trigger_location)
        self._rng = CarlaDataProvider.get_random_seed()

        self._adversary_speed = 1.5
        # Start pedestrian crossing earlier than before: 15.0m -> 19.5m (1.3x)
        self._min_trigger_dist = 15.0 * 1.3
        self._ego_end_distance = 40
        self.timeout = timeout

        # Read XML config.
        self._blocker_type = "advertisement"
        self._pedestrian_count = None
        if hasattr(config, 'other_parameters'):
            params = config.other_parameters
            if 'blocker_type' in params:
                self._blocker_type = params['blocker_type'].get('value', 'advertisement')
            if 'pedestrian_count' in params:
                try:
                    self._pedestrian_count = int(params['pedestrian_count'].get('value', '1'))
                except (TypeError, ValueError):
                    self._pedestrian_count = None

        if self._pedestrian_count is None:
            self._pedestrian_count = self._rng.randint(1, 3)
        self._pedestrian_count = max(1, min(3, self._pedestrian_count))
        lane_width = getattr(self._reference_waypoint, 'lane_width', 3.5) or 3.5
        self._lane_width = max(2.8, min(4.5, float(lane_width)))
        self._lane_half_width = self._lane_width * 0.5

        print(f"  [CONFIG] blocker_type = {self._blocker_type}")
        print(f"  [CONFIG] pedestrian_count = {self._pedestrian_count}")
        print(f"  [CONFIG] min_trigger_dist = {self._min_trigger_dist:.1f}m")
        print(f"  [CONFIG] lane_width = {self._lane_width:.2f}m")

        lateral_base = max(2.0, min(3.0, self._lane_half_width + 0.55))
        self._blocker_x_offset = 3.2
        self._blocker_spawn_x_offset = self._blocker_x_offset * 2.0
        self._blocker_y_offset = max(2.4, min(3.6, self._lane_half_width + 0.9))

        # Walker spawn offsets are in road-local coordinates:
        # x is along the road direction, y is lateral from road center.
        if self._blocker_type.lower() == "none":
            walker_templates = [
                {'x': 4.2, 'y': lateral_base, 'z': 0.3, 'yaw': 270},
                {'x': 5.0, 'y': lateral_base + 0.25, 'z': 0.3, 'yaw': 270},
                {'x': 5.8, 'y': max(2.0, lateral_base - 0.25), 'z': 0.3, 'yaw': 270},
            ]
        else:
            hidden_base_y = self._blocker_y_offset + 0.35
            hidden_base_x = self._blocker_x_offset + 1.4
            walker_templates = [
                {'x': hidden_base_x + 0.0, 'y': hidden_base_y, 'z': 0.3, 'yaw': 270},
                {'x': hidden_base_x + 0.6, 'y': hidden_base_y + 0.20, 'z': 0.3, 'yaw': 270},
                {'x': hidden_base_x + 1.2, 'y': hidden_base_y - 0.20, 'z': 0.3, 'yaw': 270},
            ]
        self._walker_data = [dict(walker_templates[i]) for i in range(self._pedestrian_count)]

        for walker_data in self._walker_data:
            walker_data['x'] += self._rng.uniform(-0.25, 0.25)
            walker_data['y'] += self._rng.uniform(-0.10, 0.10)
            walker_data['idle_time'] = self._rng.uniform(0, 0.5)
            walker_data['speed'] = self._rng.uniform(2.0, 3.0)

        super().__init__("SimplePedestrianCrossing",
                         ego_vehicles,
                         config,
                         world,
                         debug_mode,
                         criteria_enable=criteria_enable)

    def _get_walker_transform(self, wp, displacement):
        disp_x = displacement['x']
        disp_y = displacement['y']
        disp_z = displacement['z']
        disp_yaw = displacement['yaw']

        start_vec = wp.transform.get_forward_vector()
        start_right_vec = wp.transform.get_right_vector()

        spawn_loc = wp.transform.location + carla.Location(
            disp_x * start_vec.x + disp_y * start_right_vec.x,
            disp_x * start_vec.y + disp_y * start_right_vec.y,
            0  # Don't use z offset from displacement
        )

        # Get correct ground height from map waypoint
        ground_wp = self._wmap.get_waypoint(spawn_loc, project_to_road=False)
        if ground_wp:
            spawn_loc.z = ground_wp.transform.location.z + disp_z
        else:
            spawn_loc.z = wp.transform.location.z + disp_z

        spawn_rotation = carla.Rotation(
            pitch=0,  # Keep walker upright
            yaw=wp.transform.rotation.yaw + disp_yaw,
            roll=0
        )
        return carla.Transform(spawn_loc, spawn_rotation)

    def _get_spawn_displacement_candidates(self, walker_data):
        """Search for a spawn point near the desired roadside location."""
        side_sign = 1.0 if walker_data['y'] >= 0.0 else -1.0
        min_lateral = self._lane_half_width + 0.25
        desired_lateral = max(abs(walker_data['y']), min_lateral)

        if self._blocker_type.lower() != "none":
            min_lateral = max(min_lateral, self._blocker_y_offset + 0.25)
            desired_lateral = max(desired_lateral, min_lateral)
            min_forward = self._blocker_spawn_x_offset + 1.0
        else:
            min_forward = 2.8

        target_forward = max(min_forward, walker_data['x'] * 2.0)

        forward_values = []
        for value in (
            target_forward,
            target_forward - 0.6,
            target_forward - 1.2,
            target_forward - 1.8,
            target_forward - 2.4,
            target_forward - 3.0,
            target_forward - 3.6,
        ):
            value = max(min_forward, value)
            if all(abs(value - existing) > 0.05 for existing in forward_values):
                forward_values.append(value)

        lateral_values = []
        for value in (
            desired_lateral,
            desired_lateral + 0.2,
            max(min_lateral, desired_lateral - 0.2),
            desired_lateral + 0.4,
            max(min_lateral, desired_lateral - 0.4),
        ):
            if all(abs(value - existing) > 0.05 for existing in lateral_values):
                lateral_values.append(value)

        candidates = []
        for forward_value in forward_values:
            for lateral_value in lateral_values:
                candidate = dict(walker_data)
                candidate['x'] = forward_value
                candidate['y'] = side_sign * lateral_value
                candidates.append(candidate)

        return candidates

    def _get_blocker_blueprint_ids(self):
        """Return blocker blueprint ids in preferred fallback order."""
        blocker_type = (self._blocker_type or "").strip()
        if not blocker_type:
            blocker_type = "advertisement"

        preferred_id = blocker_type
        if not preferred_id.startswith("static.prop."):
            preferred_id = f"static.prop.{preferred_id}"

        fallback_ids = [
            "static.prop.busstoplb",
            "static.prop.foodcart",
            "static.prop.haybalelb",
            "static.prop.kiosk_01",
            "static.prop.dumpster02",
            "static.prop.dumpster",
            "static.prop.streetbarrier",
            "static.prop.warningaccident",
            "static.prop.trafficwarning",
            "static.prop.warningconstruction",
            "static.prop.container",
            "static.prop.advertisement",
            "static.prop.busstop",
            "static.prop.vendingmachine",
        ]

        blueprint_ids = []
        for blueprint_id in [preferred_id] + fallback_ids:
            if blueprint_id not in blueprint_ids:
                blueprint_ids.append(blueprint_id)
        return blueprint_ids

    def _get_blocker_spawn_attempts(self, blueprint_id):
        """Provide a few spawn perturbations; large props need more retries."""
        if blueprint_id == "static.prop.container":
            return [
                (0.0, 0.0, 0.0),
                (0.0, 0.0, 0.5),
                (-0.8, 0.0, 0.0),
                (-0.8, 0.0, 0.5),
                (-1.6, 0.0, 0.0),
                (-1.6, 0.0, 0.5),
                (-0.8, -0.5, 0.0),
                (-0.8, 0.5, 0.0),
                (-1.6, -0.5, 0.0),
                (-1.6, 0.5, 0.0),
                (0.0, -0.5, 0.5),
                (0.0, 0.5, 0.5),
            ]

        return [
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.5),
            (-0.6, 0.0, 0.0),
            (-0.6, 0.0, 0.5),
            (0.0, -0.4, 0.0),
            (0.0, 0.4, 0.0),
        ]

    def _initialize_actors(self, config):
        print(f"\n{'='*60}")
        print(f"  SCENARIO LOADED: SimplePedestrianCrossing")
        print(f"  Trigger location: {self._trigger_location}")
        print(f"  Number of pedestrians: {len(self._walker_data)}")
        print(f"{'='*60}\n")

        # Use trigger point as collision point
        self._collision_wp = self._reference_waypoint
        collision_location = self._collision_wp.transform.location
        print(f"  [INFO] Collision point: {collision_location}")

        # Get road direction vectors for calculating destinations
        right_vec = self._reference_waypoint.transform.get_right_vector()

        # Store walker info for behavior creation
        self._walker_info = []
        spawned_walkers = 0

        blueprint_library = self.world.get_blueprint_library()
        walker_bps = list(blueprint_library.filter('walker.pedestrian.*'))
        if not walker_bps:
            print(f"  [ERROR] No walker blueprints available")
            return

        # Spawn walkers directly relative to the trigger waypoint
        for i, walker_data in enumerate(self._walker_data):
            # Pick a random walker blueprint
            walker_bp = self._rng.choice(walker_bps)

            desired_transform = self._get_walker_transform(self._reference_waypoint, walker_data)
            print(f"  [INFO] Spawning walker {i+1} near {desired_transform.location}")

            spawn_transform = None
            walker = None
            used_walker_data = walker_data
            for candidate_index, candidate_data in enumerate(self._get_spawn_displacement_candidates(walker_data)):
                candidate_transform = self._get_walker_transform(self._reference_waypoint, candidate_data)
                for z_offset in (0.0, 0.6, 1.0):
                    try_transform = carla.Transform(
                        carla.Location(
                            candidate_transform.location.x,
                            candidate_transform.location.y,
                            candidate_transform.location.z + z_offset,
                        ),
                        candidate_transform.rotation
                    )
                    walker = self.world.try_spawn_actor(walker_bp, try_transform)
                    if walker is not None:
                        spawn_transform = try_transform
                        used_walker_data = candidate_data
                        if candidate_index > 0 or z_offset > 0.0:
                            print(f"  [INFO] Walker {i+1} using fallback spawn at {spawn_transform.location}")
                        break
                if walker is not None:
                    break

            if walker is None:
                print(f"  [ERROR] Could not spawn walker {i+1}")
                continue

            try:
                walker.set_simulate_physics(True)
            except RuntimeError:
                pass

            # Register with CarlaDataProvider
            CarlaDataProvider._carla_actor_pool[walker.id] = walker
            self.other_actors.append(walker)
            spawned_walkers += 1

            # Calculate destination (cross to the other side of the road)
            # For positive y (right side), move to left; for negative y (left side), move to right.
            side_sign = 1.0 if used_walker_data['y'] >= 0.0 else -1.0
            cross_distance = max(self._lane_width * 1.9, abs(used_walker_data['y']) * 2.0 + 1.5)
            destination = carla.Location(
                spawn_transform.location.x - right_vec.x * (side_sign * cross_distance),
                spawn_transform.location.y - right_vec.y * (side_sign * cross_distance),
                spawn_transform.location.z
            )

            # Store walker info for behavior
            self._walker_info.append({
                'walker': walker,
                'destination': destination,
                'speed': used_walker_data['speed'],
                'idle_time': used_walker_data['idle_time']
            })
            print(f"  [INFO] Walker {i+1} destination: {destination}")

        print(f"  [INFO] Successfully spawned {spawned_walkers} walkers")
        if spawned_walkers == 0:
            print("  [ERROR] No walker was spawned for SimplePedestrianCrossing")

        # Spawn obstacle (advertisement/busstop) to block view of pedestrians
        self._spawn_view_blocker()

    def _spawn_view_blocker(self):
        """Spawn a large prop to block the ego vehicle's view of pedestrians"""
        # Check if blocker is disabled
        if self._blocker_type.lower() == "none":
            print(f"  [INFO] View blocker disabled")
            return

        blueprint_library = self.world.get_blueprint_library()

        # Position the blocker between ego vehicle and pedestrians
        # Place it slightly before the first walker position (closer to ego)
        forward_vec = self._reference_waypoint.transform.get_forward_vector()
        right_vec = self._reference_waypoint.transform.get_right_vector()

        # Keep the blocker closer to the ego than the walkers, on the same roadside.
        blocker_x_offset = self._blocker_spawn_x_offset
        blocker_y_offset = self._blocker_y_offset

        blocker_loc = self._reference_waypoint.transform.location + carla.Location(
            blocker_x_offset * forward_vec.x + blocker_y_offset * right_vec.x,
            blocker_x_offset * forward_vec.y + blocker_y_offset * right_vec.y,
            0
        )

        # Get ground height
        ground_wp = self._wmap.get_waypoint(blocker_loc, project_to_road=False)
        if ground_wp:
            blocker_loc.z = ground_wp.transform.location.z
        else:
            blocker_loc.z = self._reference_waypoint.transform.location.z

        # Rotate blocker to face the road (perpendicular to road direction)
        # This makes it a wall blocking the view
        blocker_yaw = self._reference_waypoint.transform.rotation.yaw + 90  # Perpendicular to road

        blocker_transform = carla.Transform(
            blocker_loc,
            carla.Rotation(pitch=0, yaw=blocker_yaw, roll=0)
        )

        print(f"  [INFO] Spawning view blocker at {blocker_loc}")

        found_any_blueprint = False
        for candidate_id in self._get_blocker_blueprint_ids():
            bps = list(blueprint_library.filter(candidate_id))
            if not bps:
                continue

            found_any_blueprint = True
            blocker_bp = bps[0]
            print(f"  [INFO] Trying {blocker_bp.id} as view blocker")

            blocker = None
            for dx, dy, dz in self._get_blocker_spawn_attempts(blocker_bp.id):
                try_transform = carla.Transform(
                    carla.Location(
                        x=blocker_transform.location.x + dx * forward_vec.x + dy * right_vec.x,
                        y=blocker_transform.location.y + dx * forward_vec.y + dy * right_vec.y,
                        z=blocker_transform.location.z + dz,
                    ),
                    blocker_transform.rotation
                )
                blocker = self.world.try_spawn_actor(blocker_bp, try_transform)
                if blocker is not None:
                    if dx != 0.0 or dy != 0.0 or dz != 0.0:
                        print(f"  [INFO] View blocker using fallback spawn at {try_transform.location}")
                    CarlaDataProvider._carla_actor_pool[blocker.id] = blocker
                    self.other_actors.append(blocker)
                    print(f"  [INFO] View blocker spawned successfully")
                    return

            print(f"  [WARNING] Failed to spawn {blocker_bp.id}, trying next blocker asset")

        if not found_any_blueprint:
            print(f"  [WARNING] No suitable view blocker prop found")
        else:
            print(f"  [WARNING] Failed to spawn view blocker")

    def _create_behavior(self):
        sequence = py_trees.composites.Sequence(name="SimplePedestrianCrossing")

        collision_location = self._collision_wp.transform.location

        # Trigger when ego vehicle is close
        trigger_adversary = InTriggerDistanceToLocation(
            self.ego_vehicles[0], collision_location, self._min_trigger_dist)
        sequence.add_child(trigger_adversary)

        # Notification
        sequence.add_child(ScenarioNotification(
            "SCENARIO TRIGGERED: SimplePedestrianCrossing - Pedestrians crossing!"))

        # Move walkers using AI controllers (parallel execution)
        if hasattr(self, '_walker_info') and self._walker_info:
            walker_behavior = py_trees.composites.Parallel(
                policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL,
                name="WalkersCrossing")

            for i, info in enumerate(self._walker_info):
                # Create a sequence for each walker: idle -> cross
                walker_seq = py_trees.composites.Sequence(name=f"Walker{i+1}")
                walker_seq.add_child(Idle(info['idle_time']))
                walker_seq.add_child(WalkerCrossingBehavior(
                    info['walker'],
                    info['destination'],
                    info['speed'],
                    name=f"Walker{i+1}Crossing"
                ))
                walker_behavior.add_child(walker_seq)

            # Run walker movement in parallel with end condition
            end_condition = py_trees.composites.Parallel(
                policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE,
                name="EndCondition")
            end_condition.add_child(walker_behavior)
            end_condition.add_child(DriveDistance(self.ego_vehicles[0], self._ego_end_distance, name="EgoDriveDistance"))
            sequence.add_child(end_condition)
        else:
            # Fallback: just wait for ego to drive past
            sequence.add_child(DriveDistance(self.ego_vehicles[0], self._ego_end_distance, name="EndCondition"))

        return sequence

    def _create_test_criteria(self):
        if self.route_mode:
            return []
        return [CollisionTest(self.ego_vehicles[0])]

    def __del__(self):
        self.remove_all_actors()


class BicycleCrossingBehavior(py_trees.behaviour.Behaviour):
    """
    Behavior that controls an existing bicycle using set_transform.
    The bicycle crosses the road to the destination location.
    Uses direct transform manipulation for reliable movement in UE5.
    """

    def __init__(self, bicycle, destination, speed=5.0, yaw_offset=0.0, name="BicycleCrossing"):
        super().__init__(name)
        self._bicycle = bicycle
        self._destination = destination
        self._speed = speed
        self._yaw_offset = yaw_offset
        self._initialized = False
        self._sink_dist = 3.0
        self._direction = None
        self._target_yaw = None

    def initialise(self):
        """Calculate direction and prepare for cycling"""
        if self._initialized:
            return

        if self._bicycle is None:
            print(f"  [WARNING] Bicycle is None, skipping control setup")
            return

        try:
            loc = self._bicycle.get_location()
            if loc is None:
                print(f"  [WARNING] Bicycle location is None, may be destroyed")
                return

            # Calculate yaw angle to destination
            import math
            dx = self._destination.x - loc.x
            dy = self._destination.y - loc.y
            self._target_yaw = math.degrees(math.atan2(dy, dx))

            # Calculate direction vector
            length = (dx**2 + dy**2)**0.5
            if length > 0.1:
                self._direction = carla.Vector3D(dx/length, dy/length, 0)
            else:
                self._direction = carla.Vector3D(1, 0, 0)

            # Set bicycle's initial rotation to face destination
            current_transform = self._bicycle.get_transform()
            current_transform.rotation.yaw = self._target_yaw + self._yaw_offset
            self._bicycle.set_transform(current_transform)

            self._initialized = True
            print(
                f"  [INFO] Bicycle ready, speed={self._speed}, "
                f"yaw={self._target_yaw:.1f}, yaw_offset={self._yaw_offset:.1f}"
            )

        except RuntimeError as e:
            print(f"  [WARNING] Bicycle control failed: {e}")
            return

    def update(self):
        """Move bicycle by directly setting transform each tick"""
        if not self._initialized:
            return py_trees.common.Status.SUCCESS

        try:
            current_transform = self._bicycle.get_transform()
            loc = current_transform.location
            if loc is None:
                return py_trees.common.Status.SUCCESS

            # Check if reached destination
            if loc.distance(self._destination) < self._sink_dist:
                print(f"  [INFO] Bicycle reached destination")
                return py_trees.common.Status.SUCCESS

            # Move bicycle by setting new transform directly
            delta_time = 0.05
            move_distance = self._speed * delta_time

            new_loc = carla.Location(
                loc.x + self._direction.x * move_distance,
                loc.y + self._direction.y * move_distance,
                loc.z
            )

            new_transform = carla.Transform(
                new_loc,
                carla.Rotation(pitch=0, yaw=self._target_yaw + self._yaw_offset, roll=0)
            )
            self._bicycle.set_transform(new_transform)

        except RuntimeError:
            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        """Nothing to clean up"""
        pass


class SimpleBicycleCrossing(BasicScenario):
    """
    Simplified bicycle crossing scenario for UE5 maps.
    Spawns bicycles directly at the trigger point without requiring
    junctions or sidewalks.

    If native bicycle vehicle blueprints are missing (common in UE5),
    this scenario falls back to spawning a movable `static.prop.mesh`
    cyclist actor with a configurable mesh path.

    XML Parameters:
        blocker_type: Type of view blocker prop name
            - "none": No blocker
            - "advertisement": Advertisement board (default)
            - "busstop": Bus stop shelter
            - "vendingmachine": Vending machine
            - "container": Large container
            - Or any other static.prop.* name
        bicycle_blueprint: Blueprint id for fallback cyclist actor (default: static.prop.mesh)
        bicycle_mesh_path: Mesh path when using static.prop.mesh
        bicycle_mass: Mass used for static.prop.mesh (default: 20.0)
        bicycle_z_offset: Rider z offset relative to ground waypoint (default: 0.45)
        bicycle_speed: Crossing speed in m/s (default: random in [4.0, 6.0])
        bicycle_yaw_offset: Extra yaw for fallback cyclist mesh (default: 90.0)
    """

    def __init__(self, world, ego_vehicles, config, debug_mode=False, criteria_enable=True, timeout=60):
        self._wmap = CarlaDataProvider.get_map()
        self._trigger_location = config.trigger_points[0].location
        self._reference_waypoint = self._wmap.get_waypoint(self._trigger_location)
        self._rng = CarlaDataProvider.get_random_seed()

        self._adversary_speed = 5.0  # Bicycles are faster than pedestrians
        self._min_trigger_dist = 20.0  # Larger trigger distance for faster bicycles
        self._ego_end_distance = 50
        self.timeout = timeout

        # Fallback settings for UE5 where native bicycle blueprints may be unavailable.
        self._bicycle_blueprint = "static.prop.mesh"
        self._bicycle_mesh_path = "/Game/Carla/Static/Static/newass/rider.rider"
        self._bicycle_mass = 20.0
        self._bicycle_z_offset = 0.45
        self._bicycle_yaw_offset = 90.0
        self._bicycle_speed_override = None
        self._use_native_bicycle_bp = False

        # Read blocker_type from XML config (default: "advertisement")
        self._blocker_type = "advertisement"
        if hasattr(config, 'other_parameters'):
            params = config.other_parameters
            if 'blocker_type' in params:
                self._blocker_type = params['blocker_type'].get('value', 'advertisement')
            if 'bicycle_blueprint' in params:
                self._bicycle_blueprint = params['bicycle_blueprint'].get('value', self._bicycle_blueprint)
            if 'bicycle_mesh_path' in params:
                self._bicycle_mesh_path = params['bicycle_mesh_path'].get('value', self._bicycle_mesh_path)
            if 'bicycle_mass' in params:
                try:
                    self._bicycle_mass = float(params['bicycle_mass'].get('value', self._bicycle_mass))
                except (TypeError, ValueError):
                    pass
            if 'bicycle_z_offset' in params:
                try:
                    self._bicycle_z_offset = float(params['bicycle_z_offset'].get('value', self._bicycle_z_offset))
                except (TypeError, ValueError):
                    pass
            if 'bicycle_yaw_offset' in params:
                try:
                    self._bicycle_yaw_offset = float(params['bicycle_yaw_offset'].get('value', self._bicycle_yaw_offset))
                except (TypeError, ValueError):
                    pass
            if 'bicycle_speed' in params:
                try:
                    self._bicycle_speed_override = float(params['bicycle_speed'].get('value', 5.0))
                except (TypeError, ValueError):
                    pass

        print(f"  [CONFIG] blocker_type = {self._blocker_type}")
        print(f"  [CONFIG] bicycle_blueprint = {self._bicycle_blueprint}")
        print(f"  [CONFIG] bicycle_mesh_path = {self._bicycle_mesh_path}")
        print(f"  [CONFIG] bicycle_mass = {self._bicycle_mass}")
        print(f"  [CONFIG] bicycle_z_offset = {self._bicycle_z_offset}")
        print(f"  [CONFIG] bicycle_yaw_offset = {self._bicycle_yaw_offset}")
        if self._bicycle_speed_override is not None:
            print(f"  [CONFIG] bicycle_speed = {self._bicycle_speed_override}")

        # Bicycle spawn offsets (relative to road)
        self._bicycle_data = [
            {'x': 12.0, 'y': 6.0, 'z': 0.45, 'yaw': 270},
        ]

        for bicycle_data in self._bicycle_data:
            bicycle_data['idle_time'] = self._rng.uniform(0, 0.3)
            if self._bicycle_speed_override is not None:
                bicycle_data['speed'] = self._bicycle_speed_override
            else:
                bicycle_data['speed'] = self._rng.uniform(4.0, 6.0)  # 4-6 m/s for bicycles

        super().__init__("SimpleBicycleCrossing",
                         ego_vehicles,
                         config,
                         world,
                         debug_mode,
                         criteria_enable=criteria_enable)

    def _get_spawn_transform(self, wp, displacement):
        disp_x = displacement['x']
        disp_y = displacement['y']
        disp_z = self._bicycle_z_offset
        disp_yaw = displacement['yaw']

        start_vec = wp.transform.get_forward_vector()
        start_right_vec = wp.transform.get_right_vector()

        spawn_loc = wp.transform.location + carla.Location(
            disp_x * start_vec.x + disp_y * start_right_vec.x,
            disp_x * start_vec.y + disp_y * start_right_vec.y,
            0
        )

        # Get correct ground height from map waypoint
        ground_wp = self._wmap.get_waypoint(spawn_loc, project_to_road=False)
        if ground_wp:
            spawn_loc.z = ground_wp.transform.location.z + disp_z
        else:
            spawn_loc.z = wp.transform.location.z + disp_z

        spawn_rotation = carla.Rotation(
            pitch=0,
            yaw=wp.transform.rotation.yaw + disp_yaw,
            roll=0
        )
        return carla.Transform(spawn_loc, spawn_rotation)

    def _initialize_actors(self, config):
        print(f"\n{'='*60}")
        print(f"  SCENARIO LOADED: SimpleBicycleCrossing")
        print(f"  Trigger location: {self._trigger_location}")
        print(f"  Number of bicycles: {len(self._bicycle_data)}")
        print(f"{'='*60}\n")

        self._collision_wp = self._reference_waypoint
        collision_location = self._collision_wp.transform.location
        print(f"  [INFO] Collision point: {collision_location}")

        right_vec = self._reference_waypoint.transform.get_right_vector()
        self._bicycle_info = []

        blueprint_library = self.world.get_blueprint_library()

        # Find native bicycle blueprints first.
        bicycle_bps = list(blueprint_library.filter('vehicle.*bicycle*'))
        if not bicycle_bps:
            bicycle_bps = list(blueprint_library.filter('vehicle.bh.crossbike'))
        if not bicycle_bps:
            bicycle_bps = list(blueprint_library.filter('vehicle.diamondback.*'))
        if not bicycle_bps:
            bicycle_bps = list(blueprint_library.filter('vehicle.gazelle.*'))

        use_native_bicycle_bp = len(bicycle_bps) > 0
        self._use_native_bicycle_bp = use_native_bicycle_bp
        prop_bp = None
        if use_native_bicycle_bp:
            print(f"  [INFO] Available native bicycle blueprints: {[bp.id for bp in bicycle_bps]}")
        else:
            print("  [INFO] No native bicycle blueprint found, fallback to static prop cyclist")
            try:
                prop_bp = blueprint_library.find(self._bicycle_blueprint)
            except RuntimeError:
                prop_bp = None
            if prop_bp is None:
                print(f"  [ERROR] Fallback blueprint not found: {self._bicycle_blueprint}")
                return
            if prop_bp.has_attribute("mesh_path"):
                prop_bp.set_attribute("mesh_path", self._bicycle_mesh_path)
            else:
                print(f"  [ERROR] Blueprint {prop_bp.id} has no mesh_path attribute")
                return
            if prop_bp.has_attribute("mass"):
                prop_bp.set_attribute("mass", str(self._bicycle_mass))
            if prop_bp.has_attribute("role_name"):
                prop_bp.set_attribute("role_name", "scenario_cyclist")

        for i, bicycle_data in enumerate(self._bicycle_data):
            spawn_transform = self._get_spawn_transform(self._reference_waypoint, bicycle_data)
            if not use_native_bicycle_bp:
                spawn_transform.rotation.yaw += self._bicycle_yaw_offset
            print(f"  [INFO] Spawning bicycle {i+1} at {spawn_transform.location}")

            bicycle_bp = self._rng.choice(bicycle_bps) if use_native_bicycle_bp else prop_bp

            spawn_attempts = [
                (0.0, 0.0, 0.0),
                (0.0, 0.0, 0.3),
                (0.0, 0.0, 0.6),
                (0.2, 0.0, 0.3),
                (-0.2, 0.0, 0.3),
                (0.0, 0.2, 0.3),
                (0.0, -0.2, 0.3),
            ]

            # Spawn bicycle
            bicycle = None
            for dx, dy, dz in spawn_attempts:
                try_tf = carla.Transform(
                    carla.Location(
                        x=spawn_transform.location.x + dx,
                        y=spawn_transform.location.y + dy,
                        z=spawn_transform.location.z + dz
                    ),
                    spawn_transform.rotation
                )
                bicycle = self.world.try_spawn_actor(bicycle_bp, try_tf)
                if bicycle is not None:
                    break

            if bicycle is None:
                print(f"  [ERROR] Could not spawn bicycle {i+1}")
                continue

            # Register with CarlaDataProvider
            CarlaDataProvider._carla_actor_pool[bicycle.id] = bicycle
            self.other_actors.append(bicycle)

            if not use_native_bicycle_bp:
                try:
                    bicycle.set_simulate_physics(False)
                except RuntimeError:
                    pass

            # Calculate destination
            cross_distance = bicycle_data['y'] * 2 + 5
            destination = carla.Location(
                spawn_transform.location.x - right_vec.x * cross_distance,
                spawn_transform.location.y - right_vec.y * cross_distance,
                spawn_transform.location.z
            )

            self._bicycle_info.append({
                'bicycle': bicycle,
                'destination': destination,
                'speed': bicycle_data['speed'],
                'idle_time': bicycle_data['idle_time']
            })
            print(f"  [INFO] Bicycle {i+1} destination: {destination}")

        print(f"  [INFO] Successfully spawned {len(self._bicycle_info)} bicycles")

        # Spawn view blocker
        self._spawn_view_blocker()

    def _spawn_view_blocker(self):
        """Spawn a large prop to block the ego vehicle's view of bicycles"""
        if self._blocker_type.lower() == "none":
            print(f"  [INFO] View blocker disabled")
            return

        blueprint_library = self.world.get_blueprint_library()

        prop_name = f"static.prop.{self._blocker_type}"
        bps = list(blueprint_library.filter(prop_name))

        if not bps:
            print(f"  [WARNING] Blocker type '{prop_name}' not found, trying fallbacks")
            fallback_types = ['advertisement', 'busstop', 'vendingmachine', 'container']
            for fallback in fallback_types:
                bps = list(blueprint_library.filter(f"static.prop.{fallback}"))
                if bps:
                    print(f"  [INFO] Using fallback: static.prop.{fallback}")
                    break

        if not bps:
            print(f"  [WARNING] No suitable view blocker prop found")
            return

        blocker_bp = bps[0]
        print(f"  [INFO] Using {blocker_bp.id} as view blocker")

        forward_vec = self._reference_waypoint.transform.get_forward_vector()
        right_vec = self._reference_waypoint.transform.get_right_vector()

        blocker_x_offset = 8.0
        blocker_y_offset = 4.0

        blocker_loc = self._reference_waypoint.transform.location + carla.Location(
            blocker_x_offset * forward_vec.x + blocker_y_offset * right_vec.x,
            blocker_x_offset * forward_vec.y + blocker_y_offset * right_vec.y,
            0
        )

        ground_wp = self._wmap.get_waypoint(blocker_loc, project_to_road=False)
        if ground_wp:
            blocker_loc.z = ground_wp.transform.location.z
        else:
            blocker_loc.z = self._reference_waypoint.transform.location.z

        blocker_yaw = self._reference_waypoint.transform.rotation.yaw + 90

        blocker_transform = carla.Transform(
            blocker_loc,
            carla.Rotation(pitch=0, yaw=blocker_yaw, roll=0)
        )

        print(f"  [INFO] Spawning view blocker at {blocker_loc}")

        blocker = self.world.try_spawn_actor(blocker_bp, blocker_transform)
        if blocker is None:
            blocker_transform.location.z += 0.5
            blocker = self.world.try_spawn_actor(blocker_bp, blocker_transform)

        if blocker is not None:
            CarlaDataProvider._carla_actor_pool[blocker.id] = blocker
            self.other_actors.append(blocker)
            print(f"  [INFO] View blocker spawned successfully")
        else:
            print(f"  [WARNING] Failed to spawn view blocker")

    def _create_behavior(self):
        sequence = py_trees.composites.Sequence(name="SimpleBicycleCrossing")

        collision_location = self._collision_wp.transform.location

        trigger_adversary = InTriggerDistanceToLocation(
            self.ego_vehicles[0], collision_location, self._min_trigger_dist)
        sequence.add_child(trigger_adversary)

        sequence.add_child(ScenarioNotification(
            "SCENARIO TRIGGERED: SimpleBicycleCrossing - Bicycle crossing!"))

        if hasattr(self, '_bicycle_info') and self._bicycle_info:
            bicycle_behavior = py_trees.composites.Parallel(
                policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL,
                name="BicyclesCrossing")

            for i, info in enumerate(self._bicycle_info):
                bicycle_seq = py_trees.composites.Sequence(name=f"Bicycle{i+1}")
                bicycle_seq.add_child(Idle(info['idle_time']))
                yaw_offset = 0.0 if self._use_native_bicycle_bp else self._bicycle_yaw_offset
                bicycle_seq.add_child(BicycleCrossingBehavior(
                    info['bicycle'],
                    info['destination'],
                    info['speed'],
                    yaw_offset=yaw_offset,
                    name=f"Bicycle{i+1}Crossing"
                ))
                bicycle_behavior.add_child(bicycle_seq)

            end_condition = py_trees.composites.Parallel(
                policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE,
                name="EndCondition")
            end_condition.add_child(bicycle_behavior)
            end_condition.add_child(DriveDistance(self.ego_vehicles[0], self._ego_end_distance, name="EgoDriveDistance"))
            sequence.add_child(end_condition)
        else:
            sequence.add_child(DriveDistance(self.ego_vehicles[0], self._ego_end_distance, name="EndCondition"))

        return sequence

    def _create_test_criteria(self):
        if self.route_mode:
            return []
        return [CollisionTest(self.ego_vehicles[0])]

    def __del__(self):
        self.remove_all_actors()
