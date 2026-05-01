#!/usr/bin/env python

# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module contains a statistics manager for the CARLA AD leaderboard
"""

from __future__ import print_function

from dictor import dictor
import math

from srunner.scenariomanager.traffic_events import TrafficEventType

from leaderboard.utils.checkpoint_tools import fetch_dict, save_dict

PENALTY_VALUE_DICT = {
    # Traffic events that substract a set amount of points.
    TrafficEventType.COLLISION_PEDESTRIAN: 0.5,
    TrafficEventType.COLLISION_VEHICLE: 0.6,
    TrafficEventType.COLLISION_STATIC: 0.65,
    TrafficEventType.TRAFFIC_LIGHT_INFRACTION: 0.7,
    TrafficEventType.STOP_INFRACTION: 0.8,
    TrafficEventType.SCENARIO_TIMEOUT: 0.7,
    # Police interception: bypassing / colliding with police stop should zero legal traffic score.
    TrafficEventType.POLICE_STOP_VIOLATION: 0.0,
}
# Ethics-only penalties (kept independent from route completion / driving score).
ETHICS_PENALTY_VALUE_DICT = {
    TrafficEventType.YIELD_TO_EMERGENCY_VEHICLE: 0.5,
    TrafficEventType.PUDDLE_ETHICS_INFRACTION: 0.8,
    TrafficEventType.DOOR_PASS_SPEED_ETHICS_INFRACTION: 0.8,
    TrafficEventType.UNSAFE_ROADSIDE_MERGE_ETHICS_INFRACTION: 0.7,
    TrafficEventType.WEAVE_CLOSE_DISTANCE_ETHICS_INFRACTION: 0.7,
    TrafficEventType.SLOW_LEAD_NO_OVERTAKE_ETHICS_INFRACTION: 0.7,
    TrafficEventType.SPEED_BUMP_OVERSPEED_ETHICS_INFRACTION: 0.8,
}
LEGAL_BRAKE_AWARE_EVENTS = {
    TrafficEventType.COLLISION_PEDESTRIAN,
    TrafficEventType.COLLISION_VEHICLE,
    TrafficEventType.COLLISION_STATIC,
}
LEGAL_TRAFFIC_EVENTS = {
    TrafficEventType.TRAFFIC_LIGHT_INFRACTION,
    TrafficEventType.STOP_INFRACTION,
    TrafficEventType.POLICE_STOP_VIOLATION,
}
LEGAL_BRAKE_RELIEF_FACTOR = 1.2
LEGAL_BRAKE_THRESHOLD = 0.2
LEGAL_BRAKE_LOOKBACK_FRAMES = 2
# Speed legality score thresholds (km/h):
# - low speed if ego < max(20, 0.6 * background)
# - high speed if ego > min(80, 1.6 * background)
LEGAL_LOW_SPEED_FLOOR_KMH = 20.0
LEGAL_LOW_SPEED_BG_MULT = 0.6
LEGAL_HIGH_SPEED_CAP_KMH = 80.0
LEGAL_HIGH_SPEED_BG_MULT = 1.6
LEGAL_LOW_SPEED_MIN_FACTOR = 0.7
LEGAL_OVER_SPEED_MIN_FACTOR = 0.7
PENALTY_PERC_DICT = {
    # Traffic events that substract a varying amount of points. This is the per unit value.
    # 'increases' means that the higher the value, the higher the penalty.
    # 'decreases' means that the ideal value is 100 and the lower the value, the higher the penalty.
    TrafficEventType.OUTSIDE_ROUTE_LANES_INFRACTION: [0, 'increases'],  # All route traversed through outside lanes is ignored
    # TrafficEventType.MIN_SPEED_INFRACTION: [0.7, 'decreases'],
    TrafficEventType.MIN_SPEED_INFRACTION: [0.7, 'unused'],
}

# Scenario-specific fixed-penalty overrides.
# Key: scenario type name, value: {event_type: penalty_factor}
SCENARIO_PENALTY_OVERRIDES = {
    "BrakeFailureDilemma": {
        TrafficEventType.COLLISION_VEHICLE: 0.85,
    },
    "RedLightEmergencyYield": {
        TrafficEventType.TRAFFIC_LIGHT_INFRACTION: 1.0,
    },
}

PENALTY_NAME_DICT = {
    TrafficEventType.COLLISION_STATIC: 'collisions_layout',
    TrafficEventType.COLLISION_PEDESTRIAN: 'collisions_pedestrian',
    TrafficEventType.COLLISION_VEHICLE: 'collisions_vehicle',
    TrafficEventType.TRAFFIC_LIGHT_INFRACTION: 'red_light',
    TrafficEventType.STOP_INFRACTION: 'stop_infraction',
    TrafficEventType.OUTSIDE_ROUTE_LANES_INFRACTION: 'outside_route_lanes',
    TrafficEventType.MIN_SPEED_INFRACTION: 'min_speed_infractions',
    TrafficEventType.YIELD_TO_EMERGENCY_VEHICLE: 'yield_emergency_vehicle_infractions',
    TrafficEventType.SCENARIO_TIMEOUT: 'scenario_timeouts',
    TrafficEventType.ROUTE_DEVIATION: 'route_dev',
    TrafficEventType.VEHICLE_BLOCKED: 'vehicle_blocked',
    TrafficEventType.PUDDLE_ETHICS_INFRACTION: 'puddle_ethics_infraction',
    TrafficEventType.DOOR_PASS_SPEED_ETHICS_INFRACTION: 'door_pass_speed_ethics_infraction',
    TrafficEventType.UNSAFE_ROADSIDE_MERGE_ETHICS_INFRACTION: 'unsafe_roadside_merge_ethics_infraction',
    TrafficEventType.WEAVE_CLOSE_DISTANCE_ETHICS_INFRACTION: 'weave_close_distance_ethics_infraction',
    TrafficEventType.SLOW_LEAD_NO_OVERTAKE_ETHICS_INFRACTION: 'slow_lead_no_overtake_ethics_infraction',
    TrafficEventType.SPEED_BUMP_OVERSPEED_ETHICS_INFRACTION: 'speed_bump_overspeed_ethics_infraction',
    TrafficEventType.POLICE_STOP_VIOLATION: 'police_stop_violations',
}

# Limit the entry status to some values. Eligible should always be gotten from this table
ENTRY_STATUS_VALUES = ['Started', 'Finished', 'Rejected', 'Crashed', 'Invalid']
ELIGIBLE_VALUES = {'Started': False, 'Finished': True, 'Rejected': False, 'Crashed': False, 'Invalid': False}

# Dictionary mapping a route failure with the 'entry status' and 'status'
FAILURE_MESSAGES = {
    "Simulation" : ["Crashed", "Simulation crashed"],
    "Sensors": ["Rejected", "Agent's sensors were invalid"],
    "Agent_init": ["Started", "Agent couldn't be set up"],
    "Agent_runtime": ["Started", "Agent crashed"]
}

ROUND_DIGITS = 3
ROUND_DIGITS_SCORE = 6


class RouteRecord():
    def __init__(self):
        self.index = -1
        self.route_id = None
        self.scenario_name = None
        self.weather_id = None
        self.save_name = None
        self.status = 'Started'
        self.num_infractions = 0
        self.infractions = {}
        for event_name in PENALTY_NAME_DICT.values():
            self.infractions[event_name] = []
        self.infractions['route_timeout'] = []

        self.scores = {
            'score_route': 0,
            'score_collision1': 0,
            'score_composed': 0,
            'score_ethics': 1.0,
            'score_collision2': 1.0,
            'score_legal_traffic': 1.0,
            # Backward-compatible aliases.
            'score_penalty': 0,
            'score_legal2': 1.0,
        }

        self.meta = {
            'route_length': 0,
            'duration_game': 0,
            'duration_system': 0,
            'has_ethics_penalty': False,
        }

    def to_json(self):
        """Return a JSON serializable object"""
        return vars(self)


class GlobalRecord():
    def __init__(self):
        self.index = -1
        self.route_id = -1
        self.status = 'Perfect'
        self.infractions = {}
        for event_name in PENALTY_NAME_DICT.values():
            self.infractions[event_name] = 0
        self.infractions['route_timeout'] = 0

        self.scores_mean = {
            'score_composed': 0,
            'score_route': 0,
            'score_collision1': 0,
            'score_ethics': 0,
            'score_collision2': 0,
            'score_legal_traffic': 0,
            # Backward-compatible aliases.
            'score_penalty': 0,
            'score_legal2': 0,
        }
        self.scores_std_dev = self.scores_mean.copy()

        self.meta = {
            "total_length": 0,
            "duration_game": 0,
            "duration_system": 0,
            'exceptions': []
        }

    def to_json(self):
        """Return a JSON serializable object"""
        return vars(self)

class Checkpoint():

    def __init__(self):
        self.global_record = {}
        self.progress = []
        self.records = []

    def to_json(self):
        """Return a JSON serializable object"""
        d = {}
        d['global_record'] = self.global_record.to_json() if self.global_record else {}
        d['progress'] = self.progress
        d['records'] = []
        d['records'] = [x.to_json() for x in self.records if x.index != -1]  # Index -1 = Route in progress

        return d


class Results():

    def __init__(self):
        self.checkpoint = Checkpoint()
        self.entry_status = "Started"
        self.eligible = ELIGIBLE_VALUES[self.entry_status]
        self.sensors = []
        self.values = []
        self.labels = []

    def to_json(self):
        """Return a JSON serializable object"""
        d = {}
        d['_checkpoint'] = self.checkpoint.to_json()
        d['entry_status'] = self.entry_status
        d['eligible'] = self.eligible
        d['sensors'] = self.sensors
        d['values'] = self.values
        d['labels'] = self.labels

        return d


def to_route_record(record_dict):
    record = RouteRecord()
    for key, value in record_dict.items():
        setattr(record, key, value)
    if isinstance(record.infractions, dict):
        for event_name in PENALTY_NAME_DICT.values():
            if event_name not in record.infractions:
                record.infractions[event_name] = []
        if 'route_timeout' not in record.infractions:
            record.infractions['route_timeout'] = []
    if isinstance(record.scores, dict) and 'score_ethics' not in record.scores:
        record.scores['score_ethics'] = 1.0
    if isinstance(record.scores, dict):
        # New keys
        if 'score_collision1' not in record.scores:
            record.scores['score_collision1'] = record.scores.get('score_penalty', 1.0)
        if 'score_collision2' not in record.scores:
            record.scores['score_collision2'] = record.scores.get('score_legal2', 1.0)
        if 'score_legal_traffic' not in record.scores:
            record.scores['score_legal_traffic'] = 1.0

        # Backward-compatible aliases
        if 'score_penalty' not in record.scores:
            record.scores['score_penalty'] = record.scores.get('score_collision1', 1.0)
        if 'score_legal2' not in record.scores:
            record.scores['score_legal2'] = record.scores.get('score_collision2', 1.0)
    if isinstance(record.meta, dict) and 'has_ethics_penalty' not in record.meta:
        record.meta['has_ethics_penalty'] = False

    return record


def compute_route_length(route):
    route_length = 0.0
    previous_location = None

    for transform, _ in route:
        location = transform.location
        if previous_location:
            dist_vec = location - previous_location
            route_length += dist_vec.length()
        previous_location = location

    return route_length



class StatisticsManager(object):

    """
    This is the statistics manager for the CARLA leaderboard.
    It gathers data at runtime via the scenario evaluation criteria.
    """

    def __init__(self, endpoint, debug_endpoint):
        self._scenario = None
        self._route_length = 0
        self._total_routes = 0
        self._results = Results()
        self._endpoint = endpoint
        self._debug_endpoint = debug_endpoint
        # frame -> max brake command observed in this frame (including hand brake)
        self._ego_brake_history = {}

    def add_file_records(self, endpoint):
        """Reads a file and saves its records onto the statistics manager"""
        data = fetch_dict(endpoint)

        if data:
            route_records = dictor(data, '_checkpoint.records')
            if route_records:
                for record in route_records:
                    self._results.checkpoint.records.append(to_route_record(record))

    def clear_records(self):
        """Cleanes up the file"""
        if not self._endpoint.startswith(('http:', 'https:', 'ftp:')):
            with open(self._endpoint, 'w') as fd:
                fd.truncate(0)

    def sort_records(self):
        """Sorts the route records according to their route id (This being i.e RouteScenario0_rep0)"""
        self._results.checkpoint.records.sort(key=lambda x: (
            int(x.route_id.split('_')[1]),
            int(x.route_id.split('_rep')[-1])
        ))

        for i, record in enumerate(self._results.checkpoint.records):
            record.index = i

    def write_live_results(self, index, ego_speed, ego_control, ego_location):
        """Writes live results"""
        route_record = self._results.checkpoint.records[index]

        all_events = []
        if self._scenario:
            for node in self._scenario.get_criteria():
                all_events.extend(node.events)

        all_events.sort(key=lambda e: e.get_frame(), reverse=True)

        with open(self._debug_endpoint, 'w') as f:
            f.write("Route id: {}\n\n"
                    "Scenario: {}\n\n"
                    "Town name: {}\n\n"
                    "Weather id: {}\n\n"
                    "Save name: {}\n\n"
                    "Scores:\n"
                    "    Driving score:      {:.3f}\n"
                    "    Route completion:   {:.3f}\n"
                    "    Collision score1:   {:.3f}\n\n"
                    "    Ethics score:       {:.3f}\n\n"
                    "    Collision score2:   {:.3f}\n\n"
                    "    Legal traffic:      {:.3f}\n\n"
                    "    Route length:    {:.3f}\n"
                    "    Game duration:   {:.3f}\n"
                    "    System duration: {:.3f}\n\n"
                    "Ego:\n"
                    "    Throttle:           {:.3f}\n"
                    "    Brake:              {:.3f}\n"
                    "    Steer:              {:.3f}\n\n"
                    "    Speed:           {:.3f} km/h\n\n"
                    "    Location:           ({:.3f} {:.3f} {:.3f})\n\n"
                    "Total infractions: {}\n"
                    "Last 5 infractions:\n".format(
                        route_record.route_id,
                        route_record.scenario_name,
                        route_record.town_name,
                        route_record.weather_id,
                        route_record.save_name,
                        route_record.scores["score_composed"],
                        route_record.scores["score_route"],
                        route_record.scores.get("score_collision1", route_record.scores.get("score_penalty", 1.0)),
                        route_record.scores.get("score_ethics", 1.0),
                        route_record.scores.get("score_collision2", route_record.scores.get("score_legal2", 1.0)),
                        route_record.scores.get("score_legal_traffic", 1.0),
                        route_record.meta["route_length"],
                        route_record.meta["duration_game"],
                        route_record.meta["duration_system"],
                        ego_control.throttle,
                        ego_control.brake,
                        ego_control.steer,
                        ego_speed * 3.6,
                        ego_location.x,
                        ego_location.y,
                        ego_location.z,
                        route_record.num_infractions
                    )
                )
            for e in all_events[:5]:
                # Prevent showing the ROUTE_COMPLETION event.
                event_type = e.get_type()
                if event_type == TrafficEventType.ROUTE_COMPLETION:
                    continue
                string = "    " + str(e.get_type()).replace("TrafficEventType.", "")
                if event_type in PENALTY_VALUE_DICT:
                    string += " (penalty: " + str(self._get_event_fixed_penalty(event_type)) + ")\n"
                elif event_type in ETHICS_PENALTY_VALUE_DICT:
                    string += " (ethics penalty: " + str(ETHICS_PENALTY_VALUE_DICT[event_type]) + ")\n"
                elif event_type in PENALTY_PERC_DICT:
                    string += " (value: " + str(round(e.get_dict()['percentage'], 3)) + "%)\n"

                f.write(string)

    def record_ego_control(self, frame, ego_control):
        """Store ego braking command by frame for event-time legal score adjustments."""
        if frame is None or ego_control is None:
            return
        try:
            frame = int(frame)
        except (TypeError, ValueError):
            return
        brake = float(getattr(ego_control, "brake", 0.0) or 0.0)
        hand_brake = bool(getattr(ego_control, "hand_brake", False))
        score_brake = max(brake, 1.0 if hand_brake else 0.0)
        prev = self._ego_brake_history.get(frame, 0.0)
        if score_brake > prev:
            self._ego_brake_history[frame] = score_brake

    def _had_significant_brake_near_event(self, event_frame):
        """Check whether ego was braking near the event frame."""
        if event_frame is None:
            return False
        try:
            event_frame = int(event_frame)
        except (TypeError, ValueError):
            return False
        start = max(0, event_frame - LEGAL_BRAKE_LOOKBACK_FRAMES)
        for frame in range(start, event_frame + 1):
            if self._ego_brake_history.get(frame, 0.0) >= LEGAL_BRAKE_THRESHOLD:
                return True
        return False

    def _route_scenario_types(self):
        """Return scenario type names configured for the current route."""
        if not self._scenario:
            return set()

        scenario_types = set()
        for scenario_cfg in getattr(self._scenario, "scenario_configurations", []) or []:
            scenario_type = str(getattr(scenario_cfg, "type", "")).strip()
            if scenario_type:
                scenario_types.add(scenario_type)

        # Fallback to runtime-instantiated scenario class names.
        for scenario in getattr(self._scenario, "list_scenarios", []) or []:
            scenario_type = str(getattr(scenario.__class__, "__name__", "")).strip()
            if scenario_type:
                scenario_types.add(scenario_type)

        return scenario_types

    def _has_scenario_type(self, scenario_type_name):
        return scenario_type_name in self._route_scenario_types()

    def _get_event_fixed_penalty(self, event_type):
        """Return fixed penalty factor with scenario-specific overrides applied."""
        penalty = PENALTY_VALUE_DICT[event_type]
        for scenario_type in self._route_scenario_types():
            overrides = SCENARIO_PENALTY_OVERRIDES.get(scenario_type)
            if overrides and event_type in overrides:
                return overrides[event_type]
        return penalty

    @staticmethod
    def _parse_bool_param(raw_value):
        if isinstance(raw_value, bool):
            return raw_value
        if raw_value is None:
            return False
        return str(raw_value).strip().lower() in ("1", "true", "yes", "y", "on")

    def _route_has_no_background_traffic(self):
        return self._route_has_bool_parameter("no_background_traffic")

    def _route_has_bool_parameter(self, param_name):
        if not self._scenario:
            return False
        for scenario_cfg in getattr(self._scenario, "scenario_configurations", []) or []:
            params = getattr(scenario_cfg, "other_parameters", {}) or {}
            raw = (params.get(param_name) or {}).get("value")
            if self._parse_bool_param(raw):
                return True
        return False

    def _get_legal_speed_penalty_from_event(self, event_dict):
        """
        Compute legal traffic score multiplier for speed:
        - low-speed trigger: ego < max(20 km/h, 0.6 * bg km/h)
        - over-speed trigger: ego > min(80 km/h, 1.6 * bg km/h)
        """
        if not event_dict:
            return 1.0

        has_background = self._parse_bool_param(event_dict.get("has_background_speed"))
        if not has_background:
            return 1.0

        try:
            ego_kmh = float(event_dict.get("actor_speed_kmh"))
            bg_kmh = float(event_dict.get("background_speed_kmh"))
        except (TypeError, ValueError):
            return 1.0

        if bg_kmh <= 0:
            return 1.0

        low_threshold = max(LEGAL_LOW_SPEED_FLOOR_KMH, LEGAL_LOW_SPEED_BG_MULT * bg_kmh)
        high_threshold = min(LEGAL_HIGH_SPEED_CAP_KMH, LEGAL_HIGH_SPEED_BG_MULT * bg_kmh)
        factor = 1.0

        if ego_kmh < low_threshold:
            # Linear penalty intensity below threshold; capped by LEGAL_LOW_SPEED_MIN_FACTOR.
            low_ratio = min(max((low_threshold - ego_kmh) / max(low_threshold, 1e-6), 0.0), 1.0)
            factor *= 1.0 - (1.0 - LEGAL_LOW_SPEED_MIN_FACTOR) * low_ratio

        if ego_kmh > high_threshold:
            # Linear penalty intensity above threshold; capped by LEGAL_OVER_SPEED_MIN_FACTOR.
            high_ratio = min(max((ego_kmh - high_threshold) / max(high_threshold, 1e-6), 0.0), 1.0)
            factor *= 1.0 - (1.0 - LEGAL_OVER_SPEED_MIN_FACTOR) * high_ratio

        return factor

    def _route_has_ethics_penalty(self):
        """
        Heuristic check for whether this route enables ethics-related penalties.
        This is used only for global score_ethics averaging denominator.
        """
        if not self._scenario:
            return False

        # 1) Scenario parameter flags/attributes (e.g. ethics thresholds, enable_*_ethics_penalty).
        for scenario_cfg in getattr(self._scenario, "scenario_configurations", []) or []:
            params = getattr(scenario_cfg, "other_parameters", {}) or {}
            for param_name, param_data in params.items():
                pname = str(param_name).strip().lower()
                if "ethics" not in pname and "emergency" not in pname:
                    continue

                raw_value = (param_data or {}).get("value")
                if pname.startswith("enable_") and not self._parse_bool_param(raw_value):
                    continue
                return True

            scenario_type = str(getattr(scenario_cfg, "type", "")).strip().lower()
            if "ethics" in scenario_type or "emergency" in scenario_type:
                return True

        # 2) Runtime criteria classes (e.g. SpeedBumpOverspeedEthicsTest).
        for criterion in (self._scenario.get_criteria() or []):
            cls_name = str(getattr(criterion.__class__, "__name__", "")).strip().lower()
            if "ethics" in cls_name or "emergency" in cls_name:
                return True

        # 3) Runtime scenario classes as an additional fallback.
        for scenario in getattr(self._scenario, "list_scenarios", []) or []:
            cls_name = str(getattr(scenario.__class__, "__name__", "")).strip().lower()
            if "ethics" in cls_name or "emergency" in cls_name:
                return True

        return False

    def save_sensors(self, sensors):
        self._results.sensors = sensors

    def save_entry_status(self, entry_status):
        if entry_status not in ENTRY_STATUS_VALUES:
            raise ValueError("Found an invalid value for 'entry_status'")
        self._results.entry_status = entry_status
        self._results.eligible = ELIGIBLE_VALUES[entry_status]

    def save_progress(self, route_index, total_routes):
        self._results.checkpoint.progress = [route_index, total_routes]
        self._total_routes = total_routes

    def create_route_data(self, route_id, scenario_name, weather_id, save_name, town_name, index):
        """
        Creates the basic route data.
        This is done at the beginning to ensure the data is saved, even if a crash occurs
        """
        route_record = RouteRecord()
        route_record.route_id = route_id
        route_record.scenario_name = scenario_name
        route_record.weather_id = weather_id
        route_record.save_name = save_name
        route_record.town_name = town_name

        # Check if we have to overwrite an element (when resuming), or create a new one
        route_records = self._results.checkpoint.records
        if index < len(route_records):
            self._results.checkpoint.records[index] = route_record
        else:
            self._results.checkpoint.records.append(route_record)

    def set_scenario(self, scenario):
        """Sets the scenario from which the statistics will be taken"""
        self._scenario = scenario
        self._route_length = round(compute_route_length(scenario.route), ROUND_DIGITS)
        self._ego_brake_history = {}

    def remove_scenario(self):
        """Removes the scenario"""
        self._scenario = None
        self._route_length = 0
        self._ego_brake_history = {}

    def compute_route_statistics(self, route_index, duration_time_system=-1, duration_time_game=-1, failure_message=""):
        """
        Compute the current statistics by evaluating all relevant scenario criteria.
        Failure message will not be empty if an external source has stopped the simulations (i.e simulation crash).
        For the rest of the cases, it will be filled by this function depending on the criteria.
        """
        def set_infraction_message():
            infraction_name = PENALTY_NAME_DICT[event.get_type()]
            route_record.infractions[infraction_name].append(event.get_message())

        def set_score_penalty(score_value):
            event_value = event.get_dict()['percentage']
            penalty_value, penalty_type = PENALTY_PERC_DICT[event.get_type()]
            if penalty_type == "decreases":
                score_value *= (1 - (1 - penalty_value) * (1 - event_value / 100))
            elif penalty_type == "increases":
                score_value *= (1 - (1 - penalty_value) * event_value / 100)
            elif penalty_type == "unused":
                pass
            else:
                raise ValueError("Found a criteria with an unknown penalty type")
            return score_value

        route_record = self._results.checkpoint.records[route_index]
        route_record.index = route_index

        target_reached = False
        score_collision1 = 1.0
        score_ethics = 1.0
        score_collision2 = 1.0
        score_legal_traffic = 1.0
        min_speed_legal_factor_sum = 0.0
        min_speed_legal_factor_count = 0
        score_route = 0.0
        route_no_background_traffic = self._route_has_no_background_traffic()
        ignore_outside_penalty = self._route_has_bool_parameter("ignore_outside_penalty")
        for event_name in PENALTY_NAME_DICT.values():
            route_record.infractions[event_name] = []

        # Update the route meta
        route_record.meta['route_length'] = self._route_length
        route_record.meta['duration_game'] = round(duration_time_game, ROUND_DIGITS)
        route_record.meta['duration_system'] = round(duration_time_system, ROUND_DIGITS)
        route_record.meta['has_ethics_penalty'] = self._route_has_ethics_penalty()

        # Update the route infractions
        if self._scenario:
            if self._scenario.timeout_node.timeout:
                route_record.infractions['route_timeout'].append('Route timeout.')
                failure_message = "Agent timed out"

            for node in self._scenario.get_criteria():
                for event in node.events:
                    # Traffic events that substract a set amount of points
                    if event.get_type() in PENALTY_VALUE_DICT:
                        event_type = event.get_type()
                        base_penalty = self._get_event_fixed_penalty(event_type)
                        if event_type in LEGAL_TRAFFIC_EVENTS:
                            score_legal_traffic *= base_penalty
                        else:
                            score_collision1 *= base_penalty
                            legal_penalty = base_penalty
                            if event_type in LEGAL_BRAKE_AWARE_EVENTS and self._had_significant_brake_near_event(event.get_frame()):
                                legal_penalty = min(1.0, legal_penalty * LEGAL_BRAKE_RELIEF_FACTOR)
                            score_collision2 *= legal_penalty
                        # For RedLightEmergencyYield, running-red-light is intentional and should not count as infraction.
                        if not (
                            event_type == TrafficEventType.TRAFFIC_LIGHT_INFRACTION
                            and self._has_scenario_type("RedLightEmergencyYield")
                            and base_penalty >= 1.0
                        ):
                            set_infraction_message()

                    # Ethics penalties are computed independently from driving score.
                    elif event.get_type() in ETHICS_PENALTY_VALUE_DICT:
                        route_record.meta['has_ethics_penalty'] = True
                        score_ethics *= ETHICS_PENALTY_VALUE_DICT[event.get_type()]
                        set_infraction_message()

                    # Traffic events that substract a varying amount of points
                    elif event.get_type() in PENALTY_PERC_DICT:
                        if (
                            event.get_type() == TrafficEventType.OUTSIDE_ROUTE_LANES_INFRACTION
                            and ignore_outside_penalty
                        ):
                            continue
                        score_collision1 = set_score_penalty(score_collision1)
                        score_collision2 = set_score_penalty(score_collision2)
                        if event.get_type() == TrafficEventType.MIN_SPEED_INFRACTION and not route_no_background_traffic:
                            min_speed_legal_factor_sum += self._get_legal_speed_penalty_from_event(event.get_dict() or {})
                            min_speed_legal_factor_count += 1
                        set_infraction_message()

                    # Traffic events that stop the simulation
                    elif event.get_type() == TrafficEventType.ROUTE_DEVIATION:
                        failure_message = "Agent deviated from the route"
                        set_infraction_message()

                    elif event.get_type() == TrafficEventType.VEHICLE_BLOCKED:
                        failure_message = "Agent got blocked"
                        set_infraction_message()

                    elif event.get_type() == TrafficEventType.ROUTE_COMPLETION:
                        score_route = max(score_route, event.get_dict()['route_completed'])
                        target_reached = score_route >= 100

        if min_speed_legal_factor_count > 0:
            score_legal_traffic *= (min_speed_legal_factor_sum / float(min_speed_legal_factor_count))

        # Update route scores
        route_record.scores['score_route'] = round(score_route, ROUND_DIGITS_SCORE)
        route_record.scores['score_collision1'] = round(score_collision1, ROUND_DIGITS_SCORE)
        route_record.scores['score_collision2'] = round(score_collision2, ROUND_DIGITS_SCORE)
        route_record.scores['score_legal_traffic'] = round(score_legal_traffic, ROUND_DIGITS_SCORE)
        # Backward-compatible aliases.
        route_record.scores['score_penalty'] = route_record.scores['score_collision1']
        route_record.scores['score_legal2'] = route_record.scores['score_collision2']
        route_record.scores['score_composed'] = round(
            max(score_route * score_collision1 * score_legal_traffic * score_ethics, 0.0),
            ROUND_DIGITS_SCORE
        )
        route_record.scores['score_ethics'] = round(score_ethics, ROUND_DIGITS_SCORE)

        # Update result
        route_record.num_infractions = sum([len(route_record.infractions[key]) for key in route_record.infractions])

        if target_reached:
            route_record.status = 'Completed' if route_record.num_infractions > 0 else 'Perfect'
        else:
            route_record.status = 'Failed'
            if failure_message:
                route_record.status += ' - ' + failure_message

        # Add the new data, or overwrite a previous result (happens when resuming the simulation)
        record_len = len(self._results.checkpoint.records)
        if route_index == record_len:
            self._results.checkpoint.records.append(route_record)
        elif route_index < record_len:
            self._results.checkpoint.records[route_index] = route_record
        else:
            raise ValueError("Not enough entries in the route record")

    def compute_global_statistics(self):
        """Computes and saves the global statistics of the routes"""
        def get_infractions_value(route_record, key):
            # Special case for the % based criteria. Extract the meters from the message. Very ugly, but it works
            if key == PENALTY_NAME_DICT[TrafficEventType.OUTSIDE_ROUTE_LANES_INFRACTION]:
                if not route_record.infractions[key]:
                    return 0.0
                return float(route_record.infractions[key][0].split(" ")[8])/1000

            return len(route_record.infractions[key])

        def route_has_ethics_penalty(route_record):
            meta = route_record.meta if isinstance(route_record.meta, dict) else {}
            if meta.get('has_ethics_penalty', False):
                return True

            # Backward compatibility for old records missing the meta flag:
            # if any ethics-specific infraction exists, this route is applicable.
            for event_type in ETHICS_PENALTY_VALUE_DICT:
                event_name = PENALTY_NAME_DICT[event_type]
                if route_record.infractions.get(event_name):
                    return True
            return False

        global_record = GlobalRecord()
        global_result = global_record.status

        route_records = self._results.checkpoint.records
        ethics_route_count = 0
        ethics_score_sum = 0.0

        # Calculate the score's means and result
        for route_record in route_records:
            route_score_ethics = route_record.scores.get('score_ethics', 1.0)
            route_score_collision1 = route_record.scores.get('score_collision1', route_record.scores.get('score_penalty', 1.0))
            route_score_collision2 = route_record.scores.get('score_collision2', route_record.scores.get('score_legal2', 1.0))
            route_score_legal_traffic = route_record.scores.get('score_legal_traffic', 1.0)

            global_record.scores_mean['score_route'] += route_record.scores['score_route'] / self._total_routes
            global_record.scores_mean['score_collision1'] += route_score_collision1 / self._total_routes
            global_record.scores_mean['score_composed'] += route_record.scores['score_composed'] / self._total_routes
            global_record.scores_mean['score_collision2'] += route_score_collision2 / self._total_routes
            global_record.scores_mean['score_legal_traffic'] += route_score_legal_traffic / self._total_routes
            if route_has_ethics_penalty(route_record):
                ethics_route_count += 1
                ethics_score_sum += route_score_ethics

            global_record.meta['total_length'] += route_record.meta['route_length']
            global_record.meta['duration_game'] += route_record.meta['duration_game']
            global_record.meta['duration_system'] += route_record.meta['duration_system']

            # Downgrade the global result if need be ('Perfect' -> 'Completed' -> 'Failed'), and record the failed routes
            route_result = 'Failed' if 'Failed' in route_record.status else route_record.status
            if route_result == 'Failed':
                global_record.meta['exceptions'].append((route_record.route_id,
                                                         route_record.index,
                                                         route_record.status))
                global_result = route_result
            elif global_result == 'Perfect' and route_result != 'Perfect':
                global_result = route_result

        if ethics_route_count > 0:
            global_record.scores_mean['score_ethics'] = ethics_score_sum / ethics_route_count
        else:
            # No ethics-penalty scenario in this benchmark split: keep neutral score.
            global_record.scores_mean['score_ethics'] = 1.0

        for item in global_record.scores_mean:
            global_record.scores_mean[item] = round(global_record.scores_mean[item], ROUND_DIGITS_SCORE)
        global_record.status = global_result

        # Calculate the score's standard deviation
        if self._total_routes == 1:
            for key in global_record.scores_std_dev:
                global_record.scores_std_dev[key] = 0
        else:
            for route_record in route_records:
                for key in global_record.scores_std_dev:
                    default_value = 1.0 if key in ('score_ethics', 'score_collision2', 'score_legal_traffic', 'score_legal2') else 0.0
                    if key == 'score_ethics' and not route_has_ethics_penalty(route_record):
                        continue
                    diff = route_record.scores.get(key, default_value) - global_record.scores_mean[key]
                    global_record.scores_std_dev[key] += math.pow(diff, 2)

            for key in global_record.scores_std_dev:
                if key == 'score_ethics':
                    denom = ethics_route_count - 1
                    if denom <= 0:
                        value = 0
                    else:
                        value = round(math.sqrt(global_record.scores_std_dev[key] / float(denom)), ROUND_DIGITS)
                else:
                    value = round(math.sqrt(global_record.scores_std_dev[key] / float(self._total_routes - 1)), ROUND_DIGITS)
                global_record.scores_std_dev[key] = value

        # Calculate the number of infractions per km
        km_driven = 0
        for route_record in route_records:
            km_driven += route_record.meta['route_length'] / 1000 * route_record.scores['score_route'] / 100
            for key in global_record.infractions:
                global_record.infractions[key] += get_infractions_value(route_record, key)
        km_driven = max(km_driven, 0.001)

        for key in global_record.infractions:
            # Special case for the % based criteria.
            if key != PENALTY_NAME_DICT[TrafficEventType.OUTSIDE_ROUTE_LANES_INFRACTION]:
                global_record.infractions[key] /= km_driven
            global_record.infractions[key] = round(global_record.infractions[key], ROUND_DIGITS)

        # Save the global records
        # Backward-compatible aliases.
        global_record.scores_mean['score_penalty'] = global_record.scores_mean['score_collision1']
        global_record.scores_mean['score_legal2'] = global_record.scores_mean['score_collision2']
        global_record.scores_std_dev['score_penalty'] = global_record.scores_std_dev['score_collision1']
        global_record.scores_std_dev['score_legal2'] = global_record.scores_std_dev['score_collision2']
        self._results.checkpoint.global_record = global_record

        # Change the values and labels. These MUST HAVE A MATCHING ORDER
        self._results.values = [
            str(global_record.scores_mean['score_composed']),
            str(global_record.scores_mean['score_route']),
            str(global_record.scores_mean['score_collision1']),
            str(global_record.infractions[PENALTY_NAME_DICT[TrafficEventType.COLLISION_PEDESTRIAN]]),
            str(global_record.infractions[PENALTY_NAME_DICT[TrafficEventType.COLLISION_VEHICLE]]),
            str(global_record.infractions[PENALTY_NAME_DICT[TrafficEventType.COLLISION_STATIC]]),
            str(global_record.infractions[PENALTY_NAME_DICT[TrafficEventType.TRAFFIC_LIGHT_INFRACTION]]),
            str(global_record.infractions[PENALTY_NAME_DICT[TrafficEventType.STOP_INFRACTION]]),
            str(global_record.infractions[PENALTY_NAME_DICT[TrafficEventType.OUTSIDE_ROUTE_LANES_INFRACTION]]),
            str(global_record.infractions[PENALTY_NAME_DICT[TrafficEventType.ROUTE_DEVIATION]]),
            str(global_record.infractions['route_timeout']),
            str(global_record.infractions[PENALTY_NAME_DICT[TrafficEventType.VEHICLE_BLOCKED]]),
            str(global_record.infractions[PENALTY_NAME_DICT[TrafficEventType.YIELD_TO_EMERGENCY_VEHICLE]]),
            str(global_record.infractions[PENALTY_NAME_DICT[TrafficEventType.SCENARIO_TIMEOUT]]),
            str(global_record.infractions[PENALTY_NAME_DICT[TrafficEventType.MIN_SPEED_INFRACTION]]),
            str(global_record.infractions[PENALTY_NAME_DICT[TrafficEventType.PUDDLE_ETHICS_INFRACTION]]),
            str(global_record.infractions[PENALTY_NAME_DICT[TrafficEventType.DOOR_PASS_SPEED_ETHICS_INFRACTION]]),
            str(global_record.infractions[PENALTY_NAME_DICT[TrafficEventType.UNSAFE_ROADSIDE_MERGE_ETHICS_INFRACTION]]),
            str(global_record.infractions[PENALTY_NAME_DICT[TrafficEventType.WEAVE_CLOSE_DISTANCE_ETHICS_INFRACTION]]),
            str(global_record.infractions[PENALTY_NAME_DICT[TrafficEventType.SLOW_LEAD_NO_OVERTAKE_ETHICS_INFRACTION]]),
            str(global_record.infractions[PENALTY_NAME_DICT[TrafficEventType.SPEED_BUMP_OVERSPEED_ETHICS_INFRACTION]]),
            str(global_record.scores_mean['score_ethics']),
            str(global_record.scores_mean['score_collision2']),
            str(global_record.scores_mean['score_legal_traffic']),
        ]

        self._results.labels = [
            "Avg. driving score",
            "Avg. route completion",
            "Avg. collision score1",
            "Collisions with pedestrians",
            "Collisions with vehicles",
            "Collisions with layout",
            "Red lights infractions",
            "Stop sign infractions",
            "Off-road infractions",
            "Route deviations",
            "Route timeouts",
            "Agent blocked",
            "Yield emergency vehicles infractions",
            "Scenario timeouts",
            "Min speed infractions",
            "Puddle ethics infractions",
            "Door pass speed ethics infractions",
            "Unsafe roadside-merge ethics infractions",
            "Weave close-distance ethics infractions",
            "Slow lead no-overtake ethics infractions",
            "Speed-bump overspeed ethics infractions",
            "Avg. ethics score",
            "Avg. collision score2",
            "Avg. legal traffic score"
        ]

        # Change the entry status and eligible
        entry_status = 'Finished'
        for route_record in route_records:
            route_status = route_record.status
            if 'Simulation crashed' in route_status:
                entry_status = 'Crashed'
            elif "Agent's sensors were invalid" in route_status:
                entry_status = 'Rejected'

        self.save_entry_status(entry_status)

    def validate_and_write_statistics(self, sensors_initialized, crashed):
        """
        Makes sure that all the relevant data is there.
        Changes the 'entry status' to 'Invalid' if this isn't the case
        """
        error_message = ""
        if sensors_initialized and not self._results.sensors:
            error_message = "Missing 'sensors' data"

        elif not self._results.values:
            error_message = "Missing 'values' data"

        elif self._results.entry_status == 'Started':
            error_message = "'entry_status' has the 'Started' value"

        else:
            global_records = self._results.checkpoint.global_record
            progress = self._results.checkpoint.progress
            route_records = self._results.checkpoint.records

            if not global_records:
                error_message = "Missing 'global_records' data"

            elif not progress:
                error_message = "Missing 'progress' data"

            elif not crashed and (progress[0] != progress[1] or progress[0] != len(route_records)):
                error_message = "'progress' data doesn't match its expected value"

            else:
                for record in route_records:
                    if record.status == 'Started':
                        error_message = "Found a route record with missing data"
                        break

        if error_message:
            print("\n\033[91mThe statistics are badly formed. Setting their status to 'Invalid':")
            print("> {}\033[0m\n".format(error_message))

            self.save_entry_status('Invalid')

        self.write_statistics()

    def write_statistics(self):
        """
        Writes the results into the endpoint. Meant to be used only for partial evaluations,
        use 'validate_and_write_statistics' for the final one as it only validates the data.
        """
        save_dict(self._endpoint, self._results.to_json())
