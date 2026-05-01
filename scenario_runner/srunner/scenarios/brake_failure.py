#!/usr/bin/env python
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Brake Failure scenario for UE5 maps.
After reaching a trigger point, brake input is ignored (brake=0 every tick),
without changing wheel physics parameters.
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


class BrakeFailureFlagSetter(py_trees.behaviour.Behaviour):
    """Sets blackboard flags for brake-failure mode and keeps them active."""

    def __init__(self, disable_handbrake=True, name="BrakeFailureFlagSetter"):
        super().__init__(name)
        self._disable_handbrake = bool(disable_handbrake)
        self._activated = False

    def _activate_once(self):
        self._activated = True
        blackboard = py_trees.blackboard.Blackboard()
        blackboard.set("BrakeFailure_active", True, overwrite=True)
        blackboard.set("BrakeFailure_disable_handbrake", self._disable_handbrake, overwrite=True)
        print(
            "  [INFO] Brake failure activated (control-layer): disable_handbrake={}".format(
                self._disable_handbrake
            ),
            flush=True,
        )

    def update(self):
        if not self._activated:
            self._activate_once()
        # Keep running so flags stay active until scenario end.
        return py_trees.common.Status.RUNNING


class BrakeFailure(BasicScenario):
    """
    Simulate brake failure for ego vehicle after passing a trigger point.

    XML Parameters:
        trigger_radius: trigger distance in meters to activate failure (default 6.0)
        disable_handbrake: whether to also disable handbrake input (default true)
        failure_distance: distance ego should drive after failure activation (default 90.0)
    """

    def __init__(self, world, ego_vehicles, config, randomize=False,
                 debug_mode=False, criteria_enable=True, timeout=120):
        self.timeout = timeout
        self._trigger_location = config.trigger_points[0].location

        self._trigger_radius = 6.0
        self._disable_handbrake = True
        self._failure_distance = 90.0

        if hasattr(config, 'other_parameters'):
            p = config.other_parameters
            if 'trigger_radius' in p:
                self._trigger_radius = float(p['trigger_radius'].get('value', 6.0))
            if 'disable_handbrake' in p:
                self._disable_handbrake = str(p['disable_handbrake'].get('value', 'true')).strip().lower() in (
                    '1', 'true', 'yes', 'on'
                )
            if 'failure_distance' in p:
                self._failure_distance = float(p['failure_distance'].get('value', 90.0))

        self._flag_setter = None

        super().__init__(
            "BrakeFailure",
            ego_vehicles,
            config,
            world,
            debug_mode,
            criteria_enable=criteria_enable,
        )

    def _initialize_actors(self, config):
        # No external actors needed for this scenario.
        print("\n{}".format('=' * 60))
        print("  SCENARIO LOADED: BrakeFailure")
        print("  Trigger location: {}".format(self._trigger_location))
        print("  trigger_radius={:.1f}, disable_handbrake={}".format(
            self._trigger_radius, self._disable_handbrake
        ))
        print("{}\n".format('=' * 60))

    def _create_behavior(self):
        sequence = py_trees.composites.Sequence(name="BrakeFailure")

        trigger = InTriggerDistanceToLocation(
            self.ego_vehicles[0],
            self._trigger_location,
            self._trigger_radius
        )
        sequence.add_child(trigger)

        self._flag_setter = BrakeFailureFlagSetter(
            disable_handbrake=self._disable_handbrake,
        )

        run = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE,
            name="BrakeFailureRun",
        )
        run.add_child(self._flag_setter)
        run.add_child(DriveDistance(
            self.ego_vehicles[0],
            self._failure_distance,
            name="EgoDriveAfterBrakeFailure",
        ))

        sequence.add_child(run)
        return sequence

    def _create_test_criteria(self):
        if self.route_mode:
            return []
        return [CollisionTest(self.ego_vehicles[0])]

    def __del__(self):
        blackboard = py_trees.blackboard.Blackboard()
        blackboard.set("BrakeFailure_active", False, overwrite=True)
        blackboard.set("BrakeFailure_disable_handbrake", False, overwrite=True)
        self.remove_all_actors()
