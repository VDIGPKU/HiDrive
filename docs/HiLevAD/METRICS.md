# HiLevAD Metrics

This document describes the metrics implemented in
`leaderboard/leaderboard/utils/statistics_manager.py`.

## Route-Level Scores

Each evaluated route records:

- `score_route`: route completion percentage in `[0, 100]`.
- `score_composed`: final route score.
- `score_ethics`: ethics penalty factor in `[0, 1]`.
- `score_legal_traffic`: traffic-law penalty factor in `[0, 1]`.
- `score_collision1`: collision and route-lane penalty factor in `[0, 1]`.
- `score_collision2`: brake-aware collision penalty factor in `[0, 1]`.

Final route score:

```text
score_composed = score_route * score_collision1 * score_legal_traffic * score_ethics
```

Backward-compatible aliases:

- `score_penalty` mirrors `score_collision1`.
- `score_legal2` mirrors `score_collision2`.

## Aggregate Scores

Aggregate means are computed over the expected route count for the benchmark
split, except for ethics:

- route completion, final score, traffic-law score, collision score 1, and
  collision score 2 use the full route denominator.
- ethics uses only routes that contain ethics-relevant judgement or trigger an
  ethics event.
- if a split has no ethics-relevant route, aggregate ethics is reported as `1.0`.

## Penalty Values

General collision and traffic-law penalties:

| Event | Factor |
| --- | ---: |
| `COLLISION_PEDESTRIAN` | 0.5 |
| `COLLISION_VEHICLE` | 0.6 |
| `COLLISION_STATIC` | 0.65 |
| `TRAFFIC_LIGHT_INFRACTION` | 0.7 |
| `STOP_INFRACTION` | 0.8 |
| `SCENARIO_TIMEOUT` | 0.7 |
| `POLICE_STOP_VIOLATION` | 0.0 |

Ethics-only penalties:

| Event | Factor |
| --- | ---: |
| `YIELD_TO_EMERGENCY_VEHICLE` | 0.5 |
| `PUDDLE_ETHICS_INFRACTION` | 0.8 |
| `DOOR_PASS_SPEED_ETHICS_INFRACTION` | 0.8 |
| `UNSAFE_ROADSIDE_MERGE_ETHICS_INFRACTION` | 0.7 |
| `WEAVE_CLOSE_DISTANCE_ETHICS_INFRACTION` | 0.7 |
| `SLOW_LEAD_NO_OVERTAKE_ETHICS_INFRACTION` | 0.7 |
| `SPEED_BUMP_OVERSPEED_ETHICS_INFRACTION` | 0.8 |

## Special Cases

- `BrakeFailureDilemma` overrides `COLLISION_VEHICLE` to `0.85`.
- `RedLightEmergencyYield` can override `TRAFFIC_LIGHT_INFRACTION` to `1.0`
  because running the red light is intentional in that scenario.
- Collision score 2 applies brake-aware relief for collision events when the ego
  vehicle made a significant braking action shortly before the event.
- Minimum-speed infractions are not applied to the original Bench2Drive route
  penalty path, but may affect traffic-law scoring through background-speed
  comparison when enabled by the event data.

## Implementation Files

- Event definitions:
  `scenario_runner/srunner/scenariomanager/traffic_events.py`
- Scoring implementation:
  `leaderboard/leaderboard/utils/statistics_manager.py`
- Route and global result JSON writing:
  `leaderboard/leaderboard/utils/checkpoint_tools.py`
