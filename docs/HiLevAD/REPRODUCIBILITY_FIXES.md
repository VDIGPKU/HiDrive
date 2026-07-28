# Reproducibility Fixes

This note summarizes the fixes made in response to reproducibility issues found
during review.

## 1. Python ABI

The released CARLA PythonAPI wheel is built for CPython 3.13
(`carla-0.10.0-cp313-cp313-linux_x86_64.whl`). The previous Python 3.10
instruction was therefore inconsistent with the released simulator package.
`environment.yml` now uses Python 3.13 and installs `requirements-py313.txt`.

## 2. CARLA launcher name

The UE5 packaged build may ship `CarlaUnreal.sh` instead of the legacy
`CarlaUE4.sh`. The evaluator and wrapper scripts now support both names.
`scripts/prepare_carla_package.sh` also creates `CarlaUE4.sh -> CarlaUnreal.sh`
when a compatibility link is useful.

## 3. `libfoonathan_memory` soname

Some released packages contain `libfoonathan_memory-0.7.3.so` while the packaged
binary asks for `libfoonathan_memory-0.7.4.so`. The preparation script creates
the compatibility link
`libfoonathan_memory-0.7.4.so -> libfoonathan_memory-0.7.3.so` after extraction.

## 4. Table 4 reproduction

We added `tools/compute_hidrive_ability_scores.py`, `tools/make_table4.py`, and
`docs/HiLevAD/REPRODUCE_TABLE4.md`. Given per-route logs
(`route_results/route_*.json`) for each method, these scripts recompute the 30
ability scores and format the Table 4 CSV.

Third-party baseline checkpoints or implementations are not redistributed when
their licenses do not allow it. For audit-level reproduction, release the raw
route logs and the manifest used by `tools/make_table4.py`.

## Quick check

```bash
cd /path/to/HiDrive
conda env create -f environment.yml
conda activate hilevad

export CARLA_ROOT=/path/to/carla_hilevad
bash scripts/prepare_carla_package.sh "$CARLA_ROOT"
pip install "$CARLA_ROOT"/PythonAPI/carla/dist/carla-*.whl
python -c "import carla; print(carla.__file__)"
```
