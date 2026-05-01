#!/usr/bin/env python

"""
Narrow passage scenario:

Static obstacles are placed on both sides of the road near the trigger point,
forming a narrow gap that the ego must carefully drive through.

Obstacles are configurable via XML parameters.
"""

import py_trees
import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import Idle
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import DriveDistance
from srunner.scenarios.basic_scenario import BasicScenario


class NarrowPassage(BasicScenario):
    """
    Place obstacles on both sides of the road to create a narrow passage.

    XML parameters:
        obstacle_type       - blueprint id suffix, e.g. 'container' for
                              'static.prop.container' (default: constructioncone)
        num_pairs           - number of obstacle pairs along the road (default 3)
        pair_spacing        - longitudinal distance between pairs in metres (default 6)
        gap_width           - lateral gap between left and right obstacles (default 2.8)
        forward_distance    - distance from trigger to first pair (default 15)
        lateral_offset      - lateral offset of passage centre from lane centre,
                              positive = right, negative = left (default 0)
    """

    def __init__(self, world, ego_vehicles, config, randomize=False,
                 debug_mode=False, criteria_enable=True, timeout=180):
        self._world = world
        self._map = CarlaDataProvider.get_map()
        self.timeout = timeout

        p = config.other_parameters
        self._obstacle_type = p.get('obstacle_type', {}).get('value', 'constructioncone')
        self._num_pairs = int(p.get('num_pairs', {}).get('value', 3))
        self._pair_spacing = float(p.get('pair_spacing', {}).get('value', 6))
        self._gap_width = float(p.get('gap_width', {}).get('value', 2.8))
        self._forward_distance = float(p.get('forward_distance', {}).get('value', 15))
        self._lateral_offset = float(p.get('lateral_offset', {}).get('value', 0))

        self._trigger_location = config.trigger_points[0].location

        super().__init__("NarrowPassage",
                         ego_vehicles, config, world,
                         debug_mode, criteria_enable=criteria_enable)

    def _initialize_actors(self, config):
        ref_wp = self._map.get_waypoint(self._trigger_location)
        bp_name = f'static.prop.{self._obstacle_type}'

        bp_lib = self._world.get_blueprint_library()
        bp_list = list(bp_lib.filter(bp_name))
        if not bp_list:
            raise RuntimeError(f"Blueprint '{bp_name}' not found")
        blueprint = bp_list[0]

        # Walk forward along the road to place each pair
        wp = ref_wp
        remaining = self._forward_distance
        while remaining > 0:
            nxt = wp.next(min(remaining, 2.0))
            if not nxt:
                break
            wp = nxt[0]
            remaining -= 2.0

        for i in range(self._num_pairs):
            right_vec = wp.transform.get_right_vector()
            centre = wp.transform.location
            half_gap = self._gap_width / 2.0

            # Shift centre by lateral_offset (positive = right)
            cx = centre.x + self._lateral_offset * right_vec.x
            cy = centre.y + self._lateral_offset * right_vec.y

            # Left obstacle
            left_loc = carla.Location(
                x=cx - half_gap * right_vec.x,
                y=cy - half_gap * right_vec.y,
                z=centre.z + 0.3)
            left_transform = carla.Transform(left_loc, wp.transform.rotation)
            left_actor = self._world.try_spawn_actor(blueprint, left_transform)
            if left_actor:
                self.other_actors.append(left_actor)
                CarlaDataProvider._carla_actor_pool[left_actor.id] = left_actor

            # Right obstacle
            right_loc = carla.Location(
                x=cx + half_gap * right_vec.x,
                y=cy + half_gap * right_vec.y,
                z=centre.z + 0.3)
            right_transform = carla.Transform(right_loc, wp.transform.rotation)
            right_actor = self._world.try_spawn_actor(blueprint, right_transform)
            if right_actor:
                self.other_actors.append(right_actor)
                CarlaDataProvider._carla_actor_pool[right_actor.id] = right_actor

            print(f"  [NarrowPassage] Pair {i+1}: left={left_loc}, right={right_loc}")

            # Advance to next pair position
            nxt = wp.next(self._pair_spacing)
            if not nxt:
                break
            wp = nxt[0]

        print(f"  [NarrowPassage] Placed {len(self.other_actors)} obstacles "
              f"({self._num_pairs} pairs, gap={self._gap_width}m)")

    def _create_behavior(self):
        behavior = py_trees.composites.Sequence("NarrowPassage")

        # End when ego drives past all obstacles, or timeout
        total_length = self._forward_distance + self._num_pairs * self._pair_spacing + 20
        end_condition = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE,
            name="NarrowPassageEnd")
        end_condition.add_child(DriveDistance(
            self.ego_vehicles[0], total_length))
        end_condition.add_child(Idle(self.timeout))
        behavior.add_child(end_condition)
        return behavior

    def _create_test_criteria(self):
        if self.route_mode:
            return []
        return [CollisionTest(self.ego_vehicles[0])]

    def __del__(self):
        self.remove_all_actors()
