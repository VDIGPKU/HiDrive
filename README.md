# HiLevAD

High-Level Evaluation Benchmark for Autonomous Driving.

HiLevAD is a closed-loop autonomous-driving benchmark built on top of
Bench2Drive. It keeps the CARLA leaderboard-style evaluation pipeline while
adding higher-level driving scenarios, ethical interaction monitors, route
splits, and updated scoring for safety, traffic-law compliance, and social
decision making.

> Release status: this repository is being prepared for open-source release.
> Some internal scripts and legacy Bench2Drive names are intentionally kept for
> compatibility during migration.

## Highlights

- 330-route full benchmark protocol in `leaderboard/data/HLADs.xml`.
- Scenario coverage for pedestrian/bicycle crossings, braking, roadside merge,
  narrow-passage following, puddle interaction, emergency vehicles, police stop,
  camera occlusion, brake failure, wrong-way vehicles, and other high-level
  driving cases.
- A/B/C route split for reporting coarse difficulty or scenario-family results.
- Extended scoring:
  - final score
  - route completion
  - ethics score
  - traffic-law score
  - collision score 1
  - collision score 2
- Bench2Drive-compatible evaluator, route XML format, scenario runner layout,
  and agent interface.

## Repository Layout

```text
HiLevAD/
  README.md
  requirements.txt
  environment.yml
  LICENSE
  assets/                       # Figures and public documentation assets
  docs/
    HiLevAD/                    # HiLevAD release notes and protocol docs
  leaderboard/
    data/                       # Route XMLs, including HLADs.xml
    leaderboard/                # Evaluation core
    scripts/                    # Legacy Bench2Drive-style evaluation scripts
  scenario_runner/
    srunner/scenarios/          # Scenario implementations
    srunner/scenariomanager/    # Atomic criteria and traffic events
  scripts/
    hilevad_eval.sh             # HiLevAD evaluation wrapper
  tools/                        # Result merging, statistics, utilities
```

Core release files:

- `leaderboard/data/HLADs.xml`: full HiLevAD benchmark routes.
- `leaderboard/leaderboard/leaderboard_evaluator.py`: route evaluator.
- `leaderboard/leaderboard/utils/statistics_manager.py`: scoring logic.
- `scenario_runner/srunner/scenariomanager/traffic_events.py`: event taxonomy.
- `scenario_runner/srunner/scenarios/`: scenario definitions.
- `requirements.txt`: Python dependencies for the benchmark/evaluator.
- `environment.yml`: conda environment entry point.
- `docs/HiLevAD/INSTALL.md`: installation and CARLA setup.
- `docs/HiLevAD/METRICS.md`: metric and penalty definitions.
- `docs/HiLevAD/SPLITS.md`: A/B/C route split.

## Environment

HiLevAD follows the Bench2Drive/CARLA leaderboard stack. See
`docs/HiLevAD/INSTALL.md` for the full setup guide.

Recommended base requirements:

- Ubuntu Linux
- NVIDIA GPU with working Vulkan support
- Conda or virtualenv
- Python compatible with your CARLA Python API
- CARLA package compatible with the routes and maps used by this repository

Install Python dependencies:

```bash
cd /path/to/HiLevAD
conda env create -f environment.yml
conda activate hilevad
```

Alternatively:

```bash
conda create -n hilevad python=3.10 -y
conda activate hilevad
pip install -r requirements.txt
```

CARLA setup depends on your local package layout. HiLevAD currently supports a
modified packaged CARLA UE5 build. The evaluator launches `CarlaUE4.sh`, which
is also the launcher name used by some UE5 CARLA packages.

Set one of the following:

```bash
# If CarlaUE4.sh is directly under the package root:
export CARLA_ROOT=/path/to/CARLA

# If CarlaUE4.sh is under a Linux/ subdirectory:
export CARLA_ROOT=/path/to/CARLA/package/root
```

The HiLevAD wrapper will detect both `$CARLA_ROOT/CarlaUE4.sh` and
`$CARLA_ROOT/Linux/CarlaUE4.sh`.

HiLevAD requires the packaged CARLA build that matches the released benchmark
version. Download and checksum information should be provided with each release.

## Quick Start

Run one route as a smoke test:

```bash
cd /path/to/HiLevAD

export CARLA_ROOT=/path/to/CARLA_or_CARLA_package_root
export TEAM_AGENT=/path/to/your_agent.py
export TEAM_CONFIG=/path/to/your_agent_config_or_checkpoint

ROUTES_SUBSET=122 \
RUN_NAME=smoke_route_122 \
bash scripts/hilevad_eval.sh
```

The wrapper writes outputs to:

```text
results/hilevad/<RUN_NAME>/
  results.json
  live_results.txt
  viz/
```

## Full Evaluation

Run the full 330-route benchmark:

```bash
cd /path/to/HiLevAD

export CARLA_ROOT=/path/to/CARLA_or_CARLA_package_root
export TEAM_AGENT=/path/to/your_agent.py
export TEAM_CONFIG=/path/to/your_agent_config_or_checkpoint

RUN_NAME=hilevad_full_$(date +%Y%m%d_%H%M%S) \
bash scripts/hilevad_eval.sh
```

Useful environment variables:

```bash
ROUTES=/path/to/routes.xml          # default: leaderboard/data/HLADs.xml
ROUTES_SUBSET=1-10,122,285-292      # optional subset by route id
RUN_NAME=my_run                     # output directory name
RUN_DIR=/path/to/output             # overrides results/hilevad/<RUN_NAME>
PORT=2000                           # base CARLA RPC port
TM_PORT=8000                        # traffic manager port
GPU_RANK=0                          # evaluator GPU rank
RESUME=1                            # resume from checkpoint if possible
USE_EXISTING_SERVER=1               # connect to an already running CARLA server
```

Resume an interrupted run by reusing the same `RUN_DIR` and enabling `RESUME=1`:

```bash
RUN_DIR=/path/to/HiLevAD/results/hilevad/my_run \
RESUME=1 \
bash scripts/hilevad_eval.sh
```

## Metrics

HiLevAD reports route-level and aggregate metrics from
`leaderboard/leaderboard/utils/statistics_manager.py`.

Main route-level fields:

- `score_route`: route completion percentage.
- `score_composed`: final score.
- `score_ethics`: ethics penalty factor.
- `score_legal_traffic`: traffic-law penalty factor.
- `score_collision1`: collision and route-lane penalty factor.
- `score_collision2`: collision score with brake-aware relief.

Final route score:

```text
score_composed = score_route * score_collision1 * score_legal_traffic * score_ethics
```

Ethics mean is computed only over routes that contain an ethics-relevant
scenario or triggered ethics event. Routes without ethics judgement are not
added to the ethics denominator.

See `docs/HiLevAD/METRICS.md` for penalty values and metric details.

## Route Splits

The current `HLADs.xml` benchmark has 330 routes:

| Split | Routes | Description |
| --- | ---: | --- |
| A | 209 | Core safety and closed-loop driving scenarios |
| B | 75 | Interaction and ethics-heavy scenarios |
| C | 46 | Rare, adversarial, malfunction, and perception-limited scenarios |
| All | 330 | Full HiLevAD benchmark |

See `docs/HiLevAD/SPLITS.md` for the route-block mapping.

## Legacy Bench2Drive Compatibility

This repository still contains Bench2Drive-compatible scripts and names, for
example:

- `leaderboard/scripts/run_evaluation.sh`
- `leaderboard/scripts/run_evaluation_debug.sh`
- `leaderboard/data/bench2drive220.xml`
- Bench2Drive-style result JSON fields such as `score_penalty`

For release stability, prefer adding HiLevAD wrappers and documentation instead
of blindly renaming imported modules or core leaderboard classes. This avoids
breaking existing agents and evaluation scripts.

## CARLA Troubleshooting

- If CARLA exits immediately, verify Vulkan first:

```bash
vulkaninfo | head
```

- If ports are stuck, check and clean them:

```bash
lsof -i :2000
bash tools/clean_carla.sh
```

- For long evaluations, use unique `PORT`, `TM_PORT`, and `RUN_DIR` values per
  process.
- If `bind: Address already in use` appears, a previous CARLA server is still
  using that port.
- If resume starts from the wrong route, verify that the checkpoint route XML,
  route subset, and total route count match the original run.

## License

This repository inherits code and assets from Bench2Drive and CARLA leaderboard
components. Keep upstream license notices intact. New HiLevAD files should carry
the project license selected for the public release.

The original Bench2Drive repository states that its assets and code are under
the license in `LICENSE` unless specified otherwise. Review license compatibility
before publishing modified assets, generated data, or pretrained models.

## Citation

For anonymous review, HiLevAD citation metadata is intentionally omitted. The
full citation will be provided in the public release. If your work directly uses
upstream Bench2Drive or CARLA components, cite the corresponding official
publications or repositories according to their licenses.

## Acknowledgement

HiLevAD is built from Bench2Drive and the CARLA leaderboard/scenario-runner
ecosystem. We acknowledge the original authors and maintain compatibility where
possible so existing agents, route XMLs, and evaluation tooling can migrate with
minimal changes.
