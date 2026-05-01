# HiLevAD Installation

This document describes the benchmark environment only. Method-specific
agent/model dependencies, checkpoints, and model runtimes should be installed
separately by each evaluated method.

## Packaged CARLA Layout

HiLevAD uses a modified packaged CARLA UE5 build. The released package is named
similarly to:

```text
Carla-0.10.0-Linux-Shipping__rider_puddle_20260408_135427.tar.gz
```

The HiLevAD evaluator expects a CARLA launcher named `CarlaUE4.sh`. Some UE5
CARLA packages keep this launcher name for compatibility.

Supported package layouts:

```text
$CARLA_ROOT/CarlaUE4.sh
$CARLA_ROOT/Linux/CarlaUE4.sh
```

## Create the Python Environment

From the repository root:

```bash
cd /path/to/HiLevAD
conda env create -f environment.yml
conda activate hilevad
```

If you prefer manual setup:

```bash
cd /path/to/HiLevAD
conda create -n hilevad python=3.10 -y
conda activate hilevad
pip install -r requirements.txt
```

Do not install agent/model dependencies into this file unless they are required
by the benchmark itself. Keep each evaluated agent responsible for its own model
environment.

## Obtain the Packaged CARLA Build

HiLevAD requires a modified packaged CARLA build that matches the benchmark
release. The package is distributed separately from this git repository because
it is large.

Expected release artifact:

```text
Carla-0.10.0-Linux-Shipping__rider_puddle_20260408_135427.tar.gz
```

Each release should provide:

```text
CARLA_PACKAGE_URL=<release download URL>
CARLA_PACKAGE_SHA256=<sha256 checksum>
```

Download and verify:

```bash
wget -O carla_hilevad.tar.gz "$CARLA_PACKAGE_URL"
echo "$CARLA_PACKAGE_SHA256  carla_hilevad.tar.gz" | sha256sum -c -
```

Extract:

```bash
mkdir -p /path/to/carla_hilevad
tar -xzf carla_hilevad.tar.gz -C /path/to/carla_hilevad --strip-components=1
```

The extracted directory should contain one of:

```text
/path/to/carla_hilevad/CarlaUE4.sh
/path/to/carla_hilevad/Linux/CarlaUE4.sh
```

## Configure CARLA

Set `CARLA_ROOT` to the packaged CARLA root:

```bash
export CARLA_ROOT=/path/to/carla_hilevad
```

The wrapper `scripts/hilevad_eval.sh` automatically adds these paths when they
exist:

```text
$CARLA_ROOT/PythonAPI
$CARLA_ROOT/PythonAPI/carla
$CARLA_ROOT/PythonAPI/carla/dist/carla-*.egg
```

If your CARLA package installs the Python API as a `.so` in the conda
environment instead of an egg, make sure `python -c "import carla"` works in the
selected environment.

## Basic Import Check

```bash
cd /path/to/HiLevAD
conda activate hilevad
export CARLA_ROOT=/path/to/carla_hilevad

python - <<'PY'
import cv2
import numpy
import pygame
import py_trees
print("HiLevAD Python dependencies: ok")
PY
```

If `import carla` does not work before running the wrapper, it may still work
during evaluation because `scripts/hilevad_eval.sh` adds the CARLA PythonAPI to
`PYTHONPATH`.

## Smoke Evaluation

Use one route first:

```bash
cd /path/to/HiLevAD
conda activate hilevad

export CARLA_ROOT=/path/to/carla_hilevad
export TEAM_AGENT=/path/to/your_agent.py
export TEAM_CONFIG=/path/to/your_agent_config_or_checkpoint

ROUTES_SUBSET=122 \
RUN_NAME=smoke_route_122 \
bash scripts/hilevad_eval.sh
```

Outputs are written to:

```text
results/hilevad/<RUN_NAME>/
```

This directory is ignored by git.

## Full Evaluation

```bash
cd /path/to/HiLevAD
conda activate hilevad

export CARLA_ROOT=/path/to/carla_hilevad
export TEAM_AGENT=/path/to/your_agent.py
export TEAM_CONFIG=/path/to/your_agent_config_or_checkpoint

RUN_NAME=hilevad_full_$(date +%Y%m%d_%H%M%S) \
bash scripts/hilevad_eval.sh
```

Useful options:

```bash
ROUTES=/path/to/routes.xml          # default: leaderboard/data/HLADs.xml
ROUTES_SUBSET=1-10,122,285-292      # optional route ids
RUN_NAME=my_run                     # output name
RUN_DIR=/path/to/output             # explicit output directory
PORT=2000                           # CARLA RPC port
TM_PORT=8000                        # traffic manager port
GPU_RANK=0                          # GPU index for evaluator
RESUME=1                            # resume checkpoint if possible
USE_EXISTING_SERVER=1               # connect to an already running server
```

## Using an Existing CARLA Server

If you start CARLA manually:

```bash
$CARLA_ROOT/Linux/CarlaUE4.sh -vulkan -RenderOffScreen -nosound -carla-rpc-port=2000
```

Then run:

```bash
USE_EXISTING_SERVER=1 \
PORT=2000 \
TM_PORT=8000 \
bash scripts/hilevad_eval.sh
```

## Troubleshooting

Port already in use:

```bash
lsof -i :2000
bash tools/clean_carla.sh
```

Vulkan issue:

```bash
vulkaninfo | head
```

Python API mismatch:

```bash
python --version
python -c "import carla; print(carla.__file__)"
```

If CARLA was compiled for a different Python ABI, rebuild or install the matching
CARLA PythonAPI for the environment.

## CARLA Package Reproducibility

HiLevAD requires a modified packaged CARLA build that matches the benchmark
release. A release should provide these values next to the CARLA package:

```text
HiLevAD git commit:
CARLA source/fork commit:
CARLA package name:
CARLA package SHA256:
Unreal Engine version:
Python version:
OS / CUDA / NVIDIA driver:
```

For anonymous review, the CARLA package URL should not reveal author identity.
Use an anonymous file host or an anonymized release artifact and provide the
checksum in the supplementary material.
