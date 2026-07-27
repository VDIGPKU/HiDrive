# Reproducing Table 4

This document describes the artifacts needed to reproduce the HiDrive ability
scores reported in Table 4.

## What is included in this repository

The repository includes:

- `leaderboard/data/HLADs.xml`: the 330 evaluation routes.
- `scripts/hilevad_eval.sh`: the end-to-end closed-loop evaluation entry point.
- `tools/compute_hidrive_ability_scores.py`: aggregation from per-route
  `route_*.json` files to the 30 HiDrive ability scores.
- `tools/make_table4.py`: formatting multiple methods' ability scores into the
  Table 4 wide CSV layout.

## Required external artifacts

Full reproduction also requires these artifacts, which should be released next
to the code because they are too large or method-specific:

1. The HiDrive CARLA UE5 packaged build, including the CPython 3.13 CARLA
   PythonAPI wheel.
2. For each evaluated method, the exact agent code, configuration, checkpoint,
   and model-specific dependencies.
3. For audit-only reproduction of the paper numbers, the raw per-route logs:
   `route_results/route_1.json` ... `route_results/route_330.json` for each
   method.

If a baseline implementation or checkpoint is owned by a third party, the
HiDrive repository cannot redistribute it. In that case, the release should
provide the wrapper agent and document the upstream source/checkpoint used.

## Run a full evaluation

```bash
cd /path/to/HiDrive
conda activate hilevad

export CARLA_ROOT=/path/to/carla_hilevad
bash scripts/prepare_carla_package.sh "$CARLA_ROOT"

export TEAM_AGENT=/path/to/method_agent.py
export TEAM_CONFIG=/path/to/method_config_or_checkpoint

RUN_NAME=<method>_hidrive_full \
bash scripts/hilevad_eval.sh
```

The evaluator writes results to:

```text
results/hilevad/<RUN_NAME>/
```

If evaluations are run route-by-route, place the per-route JSON files under:

```text
results/hilevad/<RUN_NAME>/route_results/route_<id>.json
```

## Compute one method's 30 ability scores

```bash
python tools/compute_hidrive_ability_scores.py \
  results/hilevad/<RUN_NAME> \
  -o results/hilevad/<RUN_NAME>/ability_scores.json
```

## Recreate the Table 4 CSV

Create a manifest that maps paper table column names to each method's result
folder or precomputed `ability_scores.json`:

```json
{
  "TCP [24]": "results/hilevad/tcp_full",
  "UniAD-Base [9]": "results/hilevad/uniad_base_full",
  "VAD [14]": "results/hilevad/vad_full",
  "KnowVal [25]": "results/hilevad/knowval_full"
}
```

Then run:

```bash
python tools/make_table4.py table4_manifest.json -o table4_reproduced.csv
```

The output CSV has this layout:

```text
Set,Ability,TCP [24],UniAD-Base [9],...,KnowVal [25]
```

## Sanity checks

Before running the full benchmark, verify the CARLA package and Python ABI:

```bash
bash scripts/prepare_carla_package.sh "$CARLA_ROOT"
python --version
python -c "import carla; print(carla.__file__)"
```

For the released package, Python should be 3.13 unless a matching CARLA wheel
for another Python version is provided.
