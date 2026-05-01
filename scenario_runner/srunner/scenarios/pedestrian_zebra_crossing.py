"""Pedestrian zebra-crossing scenario."""
import carla
import py_trees
import math

from srunner.scenarios.basic_scenario import BasicScenario
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider

from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (
    ActorDestroy,
    ActorTransformSetter,
    Idle,
    KeepVelocity,
    MovePedestrianWithEgo,
)

from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (
    InTriggerDistanceToLocation,
    InTimeToArrivalToLocation,
)

from srunner.scenariomanager.scenarioatomics.atomic_criteria import (
    CollisionTest,
)


class PedestrianZebraCrossing(BasicScenario):
    """Pedestrian crossing scenario with configurable trigger timing."""

    DEFAULT_PEDESTRIAN_SPEED = 1.5
    DEFAULT_REACTION_TIME = 3.0
    DEFAULT_TRIGGER_DISTANCE = 15.0
    DEFAULT_CROSSING_DISTANCE = 12.0

    def __init__(self, world, ego_vehicles, config, debug_mode=False,
                 criteria_enable=True, timeout=60):
        """Initialize the scenario."""
        self._world = world
        self._map = CarlaDataProvider.get_map()
        self.timeout = timeout

        self._trigger_location = config.trigger_points[0].location
        self._reference_waypoint = self._map.get_waypoint(self._trigger_location)

        self._direction = "right"
        self._pedestrian_speed = self.DEFAULT_PEDESTRIAN_SPEED
        self._reaction_time = self.DEFAULT_REACTION_TIME
        self._trigger_distance = self.DEFAULT_TRIGGER_DISTANCE
        self._crossing_distance = self.DEFAULT_CROSSING_DISTANCE

        self._parse_config(config)

        self._debug = debug_mode

        self._collision_location = None

        super().__init__(
            "PedestrianZebraCrossing",
            ego_vehicles,
            config,
            world,
            debug_mode,
            criteria_enable=criteria_enable
        )

    def _parse_config(self, config):
        """Parse XML configuration parameters."""
        if hasattr(config, 'other_parameters'):
            for param in config.other_parameters:
                if 'direction' in param:
                    self._direction = param['direction'].get('value', 'right')
                if 'pedestrian_speed' in param:
                    self._pedestrian_speed = float(param['pedestrian_speed']['value'])
                if 'reaction_time' in param:
                    self._reaction_time = float(param['reaction_time']['value'])
                if 'trigger_distance' in param:
                    self._trigger_distance = float(param['trigger_distance']['value'])
                if 'crossing_distance' in param:
                    self._crossing_distance = float(param['crossing_distance']['value'])

    def _find_crosswalk_location(self):
        """Find the crossing point used as the pedestrian target."""
        collision_wp = self._reference_waypoint

        search_distance = 0
        max_search = 50

        while search_distance < max_search:
            next_wps = collision_wp.next(1.0)
            if not next_wps:
                break

            collision_wp = next_wps[0]
            search_distance += 1.0

            if collision_wp.is_junction:
                if self._debug:
                    print(f"[DEBUG] Found junction at distance {search_distance}m")
                break

        return collision_wp

    def _find_sidewalk_spawn_point(self, road_wp):
        """Find a sidewalk transform for spawning the pedestrian."""
        sidewalk_wp = road_wp

        if self._direction == "right":
            while sidewalk_wp.lane_type != carla.LaneType.Sidewalk:
                right_wp = sidewalk_wp.get_right_lane()
                if right_wp is None:
                    break
                sidewalk_wp = right_wp
        else:
            while sidewalk_wp.lane_type != carla.LaneType.Sidewalk:
                left_wp = sidewalk_wp.get_left_lane()
                if left_wp is None:
                    break
                sidewalk_wp = left_wp

        spawn_transform = sidewalk_wp.transform

        if self._direction == "right":
            spawn_transform.rotation.yaw += 90
        else:
            spawn_transform.rotation.yaw -= 90

        spawn_transform.location.z += 0.5

        return spawn_transform

    def _initialize_actors(self, config):
        """Spawn the pedestrian actor."""
        crosswalk_wp = self._find_crosswalk_location()
        self._collision_location = crosswalk_wp.transform.location

        spawn_transform = self._find_sidewalk_spawn_point(crosswalk_wp)

        if self._debug:
            print(f"[DEBUG] Crosswalk location: {self._collision_location}")
            print(f"[DEBUG] Pedestrian spawn: {spawn_transform.location}")
            self._world.debug.draw_point(
                self._collision_location,
                size=0.3,
                color=carla.Color(255, 255, 0),
                life_time=60.0
            )
            self._world.debug.draw_point(
                spawn_transform.location,
                size=0.3,
                color=carla.Color(0, 255, 0),
                life_time=60.0
            )

        pedestrian = CarlaDataProvider.request_new_actor(
            'walker.pedestrian.*',
            spawn_transform
        )

        if pedestrian is None:
            pedestrian = CarlaDataProvider.request_new_actor(
                'walker.pedestrian.0001',
                spawn_transform
            )

        if pedestrian:
            self.other_actors.append(pedestrian)
            if self._debug:
                print(f"[DEBUG] Pedestrian spawned: {pedestrian.type_id}")
        else:
            print("[WARNING] Failed to spawn pedestrian, trying alternative location")
            alt_transform = carla.Transform(
                carla.Location(
                    x=self._trigger_location.x,
                    y=self._trigger_location.y + (5 if self._direction == "right" else -5),
                    z=self._trigger_location.z + 0.5
                )
            )
            pedestrian = CarlaDataProvider.request_new_actor(
                'walker.pedestrian.0001',
                alt_transform
            )
            if pedestrian:
                self.other_actors.append(pedestrian)

    def _create_behavior(self):
        """Create the pedestrian crossing behavior tree."""
        root = py_trees.composites.Sequence("PedestrianZebraCrossing")

        if len(self.other_actors) == 0:
            return root

        pedestrian = self.other_actors[0]

        trigger = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE,
            name="WaitForTrigger"
        )

        trigger.add_child(InTimeToArrivalToLocation(
            self.ego_vehicles[0],
            self._reaction_time,
            self._collision_location,
            name="TTA_Trigger"
        ))

        trigger.add_child(InTriggerDistanceToLocation(
            self.ego_vehicles[0],
            self._collision_location,
            self._trigger_distance,
            name="Distance_Trigger"
        ))

        crossing_duration = self._crossing_distance / self._pedestrian_speed

        pedestrian_walk = KeepVelocity(
            pedestrian,
            self._pedestrian_speed,
            False,
            crossing_duration,
            self._crossing_distance,
            name="PedestrianCrossing"
        )

        destroy_pedestrian = ActorDestroy(pedestrian, name="DestroyPedestrian")

        root.add_children([
            trigger,
            pedestrian_walk,
            destroy_pedestrian
        ])

        return root

    def _create_test_criteria(self):
        """Create scenario evaluation criteria."""
        criteria = []

        collision_criterion = CollisionTest(self.ego_vehicles[0])
        criteria.append(collision_criterion)

        return criteria

    def __del__(self):
        """Clean up scenario resources."""
        self.remove_all_actors()


class PedestrianZebraCrossingFast(PedestrianZebraCrossing):
    """Faster pedestrian crossing variant."""
    DEFAULT_PEDESTRIAN_SPEED = 3.0
    DEFAULT_REACTION_TIME = 2.0


class PedestrianZebraCrossingSlow(PedestrianZebraCrossing):
    """Slower pedestrian crossing variant."""
    DEFAULT_PEDESTRIAN_SPEED = 0.8
    DEFAULT_REACTION_TIME = 4.0
