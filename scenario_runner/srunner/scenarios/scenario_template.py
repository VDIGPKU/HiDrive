"""Template for implementing custom scenarios."""
import carla
import py_trees

from srunner.scenarios.basic_scenario import BasicScenario
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider

from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (
    ActorDestroy,
    ActorTransformSetter,
    BasicAgentBehavior,
    KeepVelocity,
    LaneChange,
    StopVehicle,
    WaypointFollower,
)

from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (
    DriveDistance,
    InTriggerDistanceToLocation,
    InTriggerDistanceToVehicle,
    InTimeToArrivalToLocation,
    StandStill,
)

from srunner.scenariomanager.scenarioatomics.atomic_criteria import (
    CollisionTest,
    InRouteTest,
    OutsideRouteLanesTest,
    RouteCompletionTest,
    RunningRedLightTest,
    RunningStopTest,
)


class ScenarioTemplate(BasicScenario):
    """Template scenario class for adding custom scenarios."""

    DEFAULT_TIMEOUT = 60
    DEFAULT_NPC_SPEED = 10.0  # m/s
    DEFAULT_TRIGGER_DISTANCE = 50.0

    def __init__(
        self,
        world,
        ego_vehicles,
        config,
        debug_mode=False,
        criteria_enable=True,
        timeout=DEFAULT_TIMEOUT
    ):
        """Initialize the template scenario."""
        self._world = world
        self._map = CarlaDataProvider.get_map()

        self.timeout = timeout

        self._trigger_location = config.trigger_points[0].location

        self._parse_custom_parameters(config)

        self._debug = debug_mode

        super().__init__(
            "ScenarioTemplate",
            ego_vehicles,
            config,
            world,
            debug_mode,
            criteria_enable=criteria_enable
        )

    def _parse_custom_parameters(self, config):
        """Parse scenario-specific XML parameters.

        <scenario name="ScenarioTemplate_1" type="ScenarioTemplate">
            <trigger_point x="100" y="200" z="0.5" yaw="0" />
            <npc_speed value="15.0" />
            <trigger_distance value="40.0" />
        </scenario>
        """
        self._npc_speed = self.DEFAULT_NPC_SPEED
        self._trigger_distance = self.DEFAULT_TRIGGER_DISTANCE

        if hasattr(config, 'other_parameters'):
            for param in config.other_parameters:
                if 'npc_speed' in param:
                    self._npc_speed = float(param['npc_speed']['value'])
                if 'trigger_distance' in param:
                    self._trigger_distance = float(param['trigger_distance']['value'])

    def _initialize_actors(self, config):
        """Initialize NPC actors for the template scenario."""
        spawn_offset = carla.Location(x=30, y=0, z=0)
        spawn_location = carla.Location(
            x=self._trigger_location.x + spawn_offset.x,
            y=self._trigger_location.y + spawn_offset.y,
            z=self._trigger_location.z + spawn_offset.z
        )

        waypoint = self._map.get_waypoint(spawn_location)
        if waypoint is None:
            print(f"[WARNING] No valid waypoint found at {spawn_location}")
            return

        spawn_transform = waypoint.transform

        if self._debug:
            print(f"[DEBUG] Spawning NPC at: {spawn_transform.location}")
            self._world.debug.draw_point(
                spawn_transform.location,
                size=0.3,
                color=carla.Color(255, 0, 0),
                life_time=60.0
            )

        npc_vehicle = CarlaDataProvider.request_new_actor(
            'vehicle.tesla.model3',
            spawn_transform
        )

        if npc_vehicle:
            self.other_actors.append(npc_vehicle)
            if self._debug:
                print(f"[DEBUG] NPC vehicle spawned: {npc_vehicle.id}")
        else:
            print("[WARNING] Failed to spawn NPC vehicle")

    def _create_behavior(self):
        """Create the template scenario behavior tree."""
        root = py_trees.composites.Sequence("ScenarioTemplateBehavior")

        wait_for_trigger = InTriggerDistanceToLocation(
            self.ego_vehicles[0],
            self._trigger_location,
            distance=self._trigger_distance,
            name="WaitForEgoApproach"
        )

        if len(self.other_actors) > 0:
            npc = self.other_actors[0]

            npc_drive = KeepVelocity(
                npc,
                target_velocity=self._npc_speed,
                duration=5.0,
                name="NPCDrive"
            )

            ego_drive_distance = DriveDistance(
                self.ego_vehicles[0],
                distance=100.0,
                name="EgoDriveDistance"
            )

            destroy_npc = ActorDestroy(npc, name="DestroyNPC")

            root.add_children([
                wait_for_trigger,
                npc_drive,
                ego_drive_distance,
                destroy_npc
            ])
        else:
            ego_drive_distance = DriveDistance(
                self.ego_vehicles[0],
                distance=50.0,
                name="EgoDriveDistance"
            )
            root.add_children([
                wait_for_trigger,
                ego_drive_distance
            ])

        return root

    def _create_test_criteria(self):
        """Create template scenario evaluation criteria."""
        criteria = []

        collision_criterion = CollisionTest(self.ego_vehicles[0])
        criteria.append(collision_criterion)

        if hasattr(self, '_route') and self._route:
            route_criterion = InRouteTest(
                self.ego_vehicles[0],
                route=self._route,
                offroad_max=30
            )
            criteria.append(route_criterion)

        # criteria.append(RunningRedLightTest(self.ego_vehicles[0]))
        # criteria.append(RunningStopTest(self.ego_vehicles[0]))

        return criteria

    def __del__(self):
        """Clean up scenario actors."""
        self.remove_all_actors()


class ComplexScenarioExample(BasicScenario):
    """Example scenario with multiple NPC actors."""

    def __init__(self, world, ego_vehicles, config, debug_mode=False,
                 criteria_enable=True, timeout=90):
        self._world = world
        self._map = CarlaDataProvider.get_map()
        self.timeout = timeout
        self._trigger_location = config.trigger_points[0].location

        super().__init__(
            "ComplexScenarioExample",
            ego_vehicles,
            config,
            world,
            debug_mode,
            criteria_enable=criteria_enable
        )

    def _initialize_actors(self, config):
        """Spawn multiple NPC actors."""
        vehicle_wp = self._map.get_waypoint(
            carla.Location(
                x=self._trigger_location.x + 40,
                y=self._trigger_location.y + 3.5,
                z=self._trigger_location.z
            )
        )
        if vehicle_wp:
            vehicle = CarlaDataProvider.request_new_actor(
                'vehicle.audi.a2',
                vehicle_wp.transform
            )
            if vehicle:
                self.other_actors.append(vehicle)

        pedestrian_location = carla.Location(
            x=self._trigger_location.x + 60,
            y=self._trigger_location.y + 8,
            z=self._trigger_location.z + 1
        )
        pedestrian = CarlaDataProvider.request_new_actor(
            'walker.pedestrian.0001',
            carla.Transform(pedestrian_location)
        )
        if pedestrian:
            self.other_actors.append(pedestrian)

    def _create_behavior(self):
        """Create a parallel behavior tree."""
        root = py_trees.composites.Sequence("ComplexScenario")

        trigger = InTriggerDistanceToLocation(
            self.ego_vehicles[0],
            self._trigger_location,
            distance=60.0
        )

        parallel_behaviors = py_trees.composites.Parallel(
            "ParallelNPCBehaviors",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL
        )

        if len(self.other_actors) >= 2:
            vehicle = self.other_actors[0]
            pedestrian = self.other_actors[1]

            vehicle_behavior = py_trees.composites.Sequence("VehicleBehavior")
            vehicle_behavior.add_children([
                KeepVelocity(vehicle, 12.0, duration=2.0),
                LaneChange(vehicle, speed=12.0, direction='left',
                          distance_same_lane=5, distance_other_lane=20)
            ])

            pedestrian_behavior = py_trees.composites.Sequence("PedestrianBehavior")
            pedestrian_destination = carla.Location(
                x=self._trigger_location.x + 60,
                y=self._trigger_location.y - 8,
                z=self._trigger_location.z
            )
            pedestrian_behavior.add_children([
                BasicAgentBehavior(pedestrian, pedestrian_destination)
            ])

            parallel_behaviors.add_children([
                vehicle_behavior,
                pedestrian_behavior
            ])

        ego_pass = DriveDistance(self.ego_vehicles[0], 80.0)

        cleanup = py_trees.composites.Parallel(
            "Cleanup",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL
        )
        for actor in self.other_actors:
            cleanup.add_child(ActorDestroy(actor))

        root.add_children([trigger, parallel_behaviors, ego_pass, cleanup])
        return root

    def _create_test_criteria(self):
        criteria = [
            CollisionTest(self.ego_vehicles[0]),
        ]
        if hasattr(self, '_route') and self._route:
            criteria.append(InRouteTest(
                self.ego_vehicles[0],
                route=self._route,
                offroad_max=30
            ))
        return criteria
