#!/usr/bin/env python

"""
Slow lead vehicle scenario for overtaking:

A single vehicle is spawned ahead of the ego on the same lane from the very
start, driving slowly via autopilot + TrafficManager speed reduction.
The ego must overtake it.
"""

import py_trees
import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import Idle
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest, Criterion
from srunner.scenariomanager.timer import GameTime
from srunner.scenariomanager.traffic_events import TrafficEvent, TrafficEventType
from srunner.scenarios.basic_scenario import BasicScenario


def _safe_set_autopilot(actor, enabled):
    """Best-effort autopilot toggle using the active TM port."""
    if actor is None:
        return False
    if hasattr(actor, "is_alive") and not actor.is_alive:
        return False

    tm_port = CarlaDataProvider.get_traffic_manager_port()
    try:
        actor.set_autopilot(enabled, tm_port)
        return True
    except (RuntimeError, TypeError):
        pass

    try:
        actor.set_autopilot(enabled)
        return True
    except RuntimeError as exc:
        action = "enable" if enabled else "disable"
        print(f"[SlowLeadVehicle] WARN: failed to {action} autopilot: {exc}")
        return False


class SlowLeadNoOvertakeEthicsTest(Criterion):
    """Emit ethics infraction if ego never overtakes the slow lead vehicle."""

    def __init__(
        self,
        actor,
        lead_actor,
        monitor_duration_s=180.0,
        overtake_margin_m=2.0,
        name="SlowLeadNoOvertakeEthicsTest",
    ):
        super().__init__(name, actor, optional=True)
        self._lead_actor = lead_actor
        self._monitor_duration_s = max(0.0, float(monitor_duration_s))
        self._overtake_margin_m = max(0.1, float(overtake_margin_m))
        self._start_time = None
        self._reported = False
        self._overtake_completed = False
        self._max_longitudinal_m = float("-inf")

    def _compute_longitudinal_ahead(self):
        if self.actor is None or self._lead_actor is None:
            return None
        ego_location = CarlaDataProvider.get_location(self.actor)
        lead_location = CarlaDataProvider.get_location(self._lead_actor)
        if ego_location is None or lead_location is None:
            return None

        lead_forward = self._lead_actor.get_transform().get_forward_vector()
        rel = ego_location - lead_location
        return rel.x * lead_forward.x + rel.y * lead_forward.y

    def _report_infraction(self, reason):
        if self._reported:
            return
        self._reported = True
        self.test_status = "FAILURE"
        self.actual_value += 1

        max_longitudinal = self._max_longitudinal_m
        if max_longitudinal == float("-inf"):
            max_longitudinal = -999.0

        event = TrafficEvent(
            event_type=TrafficEventType.SLOW_LEAD_NO_OVERTAKE_ETHICS_INFRACTION,
            frame=GameTime.get_frame(),
        )
        event.set_dict({
            "max_longitudinal_ahead_m": round(max_longitudinal, 3),
            "required_overtake_margin_m": round(self._overtake_margin_m, 3),
            "monitor_duration_s": round(self._monitor_duration_s, 3),
            "reason": reason,
        })
        event.set_message(
            "SlowLeadVehicle ethics infraction: ego did not complete overtake "
            "(max ahead {:.3f} m < required {:.3f} m, reason: {})".format(
                max_longitudinal, self._overtake_margin_m, reason
            )
        )
        self.events.append(event)

    def update(self):
        new_status = py_trees.common.Status.RUNNING

        if self._reported:
            return new_status

        if self._start_time is None:
            self._start_time = GameTime.get_time()

        longitudinal = self._compute_longitudinal_ahead()
        if longitudinal is not None:
            self._max_longitudinal_m = max(self._max_longitudinal_m, longitudinal)
            if longitudinal > self._overtake_margin_m:
                self._overtake_completed = True
                self.test_status = "SUCCESS"
                return new_status

        elapsed = GameTime.get_time() - self._start_time
        if elapsed >= self._monitor_duration_s and not self._overtake_completed:
            self._report_infraction("monitor_timeout")

        return new_status

    def terminate(self, new_status):
        # Route/scenario may end before monitor timeout. If no overtake was seen,
        # still report the ethics infraction here as a final decision.
        if not self._overtake_completed and not self._reported:
            self._report_infraction("scenario_terminated")
        elif self.test_status in ("INIT", "RUNNING"):
            self.test_status = "SUCCESS"
        super().terminate(new_status)


class SlowLeadVehicle(BasicScenario):
    """
    XML parameters:
        forward_distance       - spawn distance ahead of ego (default 20)
        lead_speed             - target speed in km/h (default 15)
        ethics_overtake_margin - ego ahead distance over lead to count as overtake (m, default 2.0)
        ethics_monitor_duration - seconds to monitor overtake before penalizing (default scenario timeout)
    """

    def __init__(self, world, ego_vehicles, config, randomize=False,
                 debug_mode=False, criteria_enable=True, timeout=180):
        self._world = world
        self._map = CarlaDataProvider.get_map()
        self.timeout = timeout

        p = config.other_parameters
        self._forward_distance = float(p.get('forward_distance', {}).get('value', 20))
        self._lead_speed = float(p.get('lead_speed', {}).get('value', 15))
        self._ethics_overtake_margin_m = float(p.get('ethics_overtake_margin', {}).get('value', 2.0))
        self._ethics_monitor_duration_s = float(p.get('ethics_monitor_duration', {}).get('value', self.timeout))

        super().__init__("SlowLeadVehicle",
                         ego_vehicles, config, world,
                         debug_mode, criteria_enable=criteria_enable)

    def _initialize_actors(self, config):
        ego_wp = self._map.get_waypoint(self.ego_vehicles[0].get_location())
        front_wps = ego_wp.next(self._forward_distance)
        if not front_wps:
            raise RuntimeError("Cannot find waypoint ahead of ego")

        lead_transform = front_wps[0].transform
        lead_transform.location.z += 0.5

        self._lead_vehicle = CarlaDataProvider.request_new_actor(
            'vehicle.*', lead_transform, rolename='scenario',
            attribute_filter={'base_type': 'car', 'has_lights': True})
        if self._lead_vehicle is None:
            raise RuntimeError("Could not spawn lead vehicle")
        self.other_actors.append(self._lead_vehicle)

        self._lead_vehicle.set_transform(lead_transform)
        self._lead_vehicle.set_simulate_physics(True)
        _safe_set_autopilot(self._lead_vehicle, True)

        # Keep the lead vehicle at a fixed low speed without route-dependent acceleration.
        tm = CarlaDataProvider.get_client().get_trafficmanager(
            CarlaDataProvider.get_traffic_manager_port()
        )
        tm.auto_lane_change(self._lead_vehicle, False)
        speed_mode = ""
        try:
            # Preferred mode: fixed absolute target speed in km/h.
            tm.set_desired_speed(self._lead_vehicle, self._lead_speed)
            speed_mode = f"TM desired speed={self._lead_speed:.1f} km/h"
        except RuntimeError:
            # Fallback for older TM builds without stable desired-speed behavior.
            speed_limit = max(self._lead_vehicle.get_speed_limit(), 1.0)
            diff = max(0.0, min(100.0, 100.0 * (1.0 - self._lead_speed / speed_limit)))
            tm.vehicle_percentage_speed_difference(self._lead_vehicle, diff)
            speed_mode = f"TM speed diff={diff:.1f}% (limit={speed_limit:.1f} km/h)"

        # Give it the target velocity immediately to avoid a visible speed-up phase.
        target_speed_mps = max(0.0, self._lead_speed) / 3.6
        fwd = self._lead_vehicle.get_transform().get_forward_vector()
        try:
            self._lead_vehicle.set_target_velocity(
                carla.Vector3D(
                    x=fwd.x * target_speed_mps,
                    y=fwd.y * target_speed_mps,
                    z=0.0,
                )
            )
        except RuntimeError:
            pass

        print(f"  [SlowLeadVehicle] Spawned at {lead_transform.location}, "
              f"target speed={self._lead_speed} km/h ({speed_mode})")

    def _create_behavior(self):
        behavior = py_trees.composites.Sequence("SlowLeadVehicle")
        behavior.add_child(Idle(self.timeout))
        return behavior

    def _create_test_criteria(self):
        criteria = [
            SlowLeadNoOvertakeEthicsTest(
                self.ego_vehicles[0],
                self._lead_vehicle,
                monitor_duration_s=self._ethics_monitor_duration_s,
                overtake_margin_m=self._ethics_overtake_margin_m,
            )
        ]
        if not self.route_mode:
            criteria.append(CollisionTest(self.ego_vehicles[0]))
        return criteria

    def __del__(self):
        self.remove_all_actors()
