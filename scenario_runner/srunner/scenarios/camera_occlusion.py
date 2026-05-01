#!/usr/bin/env python
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Camera Occlusion scenario for UE5 maps.
After a trigger point, applies a rectangular black occlusion to camera images.
"""

from __future__ import print_function

import py_trees
import carla

from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (
    DriveDistance,
    InTriggerDistanceToLocation,
)
from srunner.scenarios.basic_scenario import BasicScenario


class CameraOcclusionFlagSetter(py_trees.behaviour.Behaviour):
    """Set camera occlusion flags on the blackboard and keep active."""

    def __init__(self, params, name="CameraOcclusionFlagSetter"):
        super().__init__(name)
        self._params = params
        self._activated = False

    def update(self):
        if not self._activated:
            bb = py_trees.blackboard.Blackboard()
            bb.set("CameraOcclusion_active", True, overwrite=True)
            bb.set("CameraOcclusion_tags", self._params["tags"], overwrite=True)
            bb.set("CameraOcclusion_x_ratio", self._params["x_ratio"], overwrite=True)
            bb.set("CameraOcclusion_y_ratio", self._params["y_ratio"], overwrite=True)
            bb.set("CameraOcclusion_w_ratio", self._params["w_ratio"], overwrite=True)
            bb.set("CameraOcclusion_h_ratio", self._params["h_ratio"], overwrite=True)
            self._activated = True
            print(
                "  [INFO] Camera occlusion activated: tags='{}', x={:.2f}, y={:.2f}, w={:.2f}, h={:.2f}".format(
                    self._params["tags"],
                    self._params["x_ratio"],
                    self._params["y_ratio"],
                    self._params["w_ratio"],
                    self._params["h_ratio"],
                ),
                flush=True,
            )
        return py_trees.common.Status.RUNNING


class CameraOcclusion(BasicScenario):
    """
    Trigger a camera black patch while driving.

    XML Parameters:
        trigger_radius: trigger distance in meters (default 6.0)
        effect_distance: drive distance with occlusion active (default 35.0)
        camera_tags: comma separated tags to affect, e.g. Center or Center,Left (default Center)
        x_ratio: left position ratio of occlusion rectangle [0,1] (default 0.55)
        y_ratio: top position ratio of occlusion rectangle [0,1] (default 0.20)
        w_ratio: width ratio of occlusion rectangle [0,1] (default 0.25)
        h_ratio: height ratio of occlusion rectangle [0,1] (default 0.35)
    """

    def __init__(self, world, ego_vehicles, config, randomize=False,
                 debug_mode=False, criteria_enable=True, timeout=120):
        self.timeout = timeout
        self._trigger_location = config.trigger_points[0].location

        self._trigger_radius = 6.0
        self._effect_distance = 35.0
        self._params = {
            "tags": "Center",
            "x_ratio": 0.55,
            "y_ratio": 0.20,
            "w_ratio": 0.25,
            "h_ratio": 0.35,
        }

        if hasattr(config, 'other_parameters'):
            p = config.other_parameters
            if 'trigger_radius' in p:
                self._trigger_radius = float(p['trigger_radius'].get('value', 6.0))
            if 'effect_distance' in p:
                self._effect_distance = float(p['effect_distance'].get('value', 35.0))
            if 'camera_tags' in p:
                self._params["tags"] = str(p['camera_tags'].get('value', "Center"))
            if 'x_ratio' in p:
                self._params["x_ratio"] = float(p['x_ratio'].get('value', 0.55))
            if 'y_ratio' in p:
                self._params["y_ratio"] = float(p['y_ratio'].get('value', 0.20))
            if 'w_ratio' in p:
                self._params["w_ratio"] = float(p['w_ratio'].get('value', 0.25))
            if 'h_ratio' in p:
                self._params["h_ratio"] = float(p['h_ratio'].get('value', 0.35))

        self._flag_setter = None

        super().__init__(
            "CameraOcclusion",
            ego_vehicles,
            config,
            world,
            debug_mode,
            criteria_enable=criteria_enable,
        )

    def _initialize_actors(self, config):
        print("\n{}".format('=' * 60))
        print("  SCENARIO LOADED: CameraOcclusion")
        print("  Trigger location: {}".format(self._trigger_location))
        print("  trigger_radius={:.1f}, effect_distance={:.1f}".format(
            self._trigger_radius, self._effect_distance
        ))
        print("{}\n".format('=' * 60))

    def _create_behavior(self):
        sequence = py_trees.composites.Sequence(name="CameraOcclusion")

        trigger = InTriggerDistanceToLocation(
            self.ego_vehicles[0],
            self._trigger_location,
            self._trigger_radius
        )
        sequence.add_child(trigger)

        self._flag_setter = CameraOcclusionFlagSetter(self._params)

        run = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE,
            name="CameraOcclusionRun",
        )
        run.add_child(self._flag_setter)
        run.add_child(DriveDistance(
            self.ego_vehicles[0],
            self._effect_distance,
            name="EgoDriveWithOcclusion",
        ))

        sequence.add_child(run)
        return sequence

    def _create_test_criteria(self):
        if self.route_mode:
            return []
        return [CollisionTest(self.ego_vehicles[0])]

    def __del__(self):
        bb = py_trees.blackboard.Blackboard()
        bb.set("CameraOcclusion_active", False, overwrite=True)
        bb.set("CameraOcclusion_tags", "", overwrite=True)
        self.remove_all_actors()

