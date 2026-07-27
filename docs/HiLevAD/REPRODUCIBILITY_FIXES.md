# Reproducibility Fixes / 复现问题修复说明

This note summarizes the fixes made in response to reproducibility issues found
during review. 本文档概述针对审稿复现问题所做的修复。

## 1. Python ABI / Python 版本 ABI

**English.** The released CARLA PythonAPI wheel is built for CPython 3.13
(`carla-0.10.0-cp313-cp313-linux_x86_64.whl`). The previous Python 3.10
instruction was therefore inconsistent with the released simulator package.
`environment.yml` now uses Python 3.13 and installs `requirements-py313.txt`.

**中文。** 已发布的 CARLA PythonAPI wheel 是 CPython 3.13 ABI
（`carla-0.10.0-cp313-cp313-linux_x86_64.whl`）。因此原先文档中的
Python 3.10 要求与发布的仿真器包不一致。现在 `environment.yml` 已改为
Python 3.13，并默认安装 `requirements-py313.txt`。

## 2. CARLA launcher name / CARLA 启动脚本名称

**English.** The UE5 packaged build may ship `CarlaUnreal.sh` instead of the
legacy `CarlaUE4.sh`. The evaluator and wrapper scripts now support both names.
`scripts/prepare_carla_package.sh` also creates `CarlaUE4.sh -> CarlaUnreal.sh`
when a compatibility link is useful.

**中文。** UE5 packaged build 可能只包含 `CarlaUnreal.sh`，而不是旧的
`CarlaUE4.sh`。现在 evaluator 和 wrapper 脚本都同时支持这两个名称。
`scripts/prepare_carla_package.sh` 会在需要时创建兼容链接
`CarlaUE4.sh -> CarlaUnreal.sh`。

## 3. `libfoonathan_memory` soname / 动态库 soname

**English.** Some released packages contain `libfoonathan_memory-0.7.3.so` while
the packaged binary asks for `libfoonathan_memory-0.7.4.so`. The preparation
script creates the compatibility link
`libfoonathan_memory-0.7.4.so -> libfoonathan_memory-0.7.3.so` after extraction.

**中文。** 部分发布包中包含 `libfoonathan_memory-0.7.3.so`，但二进制运行时
查找 `libfoonathan_memory-0.7.4.so`。准备脚本会在解压后创建兼容链接
`libfoonathan_memory-0.7.4.so -> libfoonathan_memory-0.7.3.so`。

## 4. Table 4 reproduction / Table 4 复现

**English.** We added `tools/compute_hidrive_ability_scores.py`,
`tools/make_table4.py`, and `docs/HiLevAD/REPRODUCE_TABLE4.md`. Given per-route
logs (`route_results/route_*.json`) for each method, these scripts recompute the
30 ability scores and format the Table 4 CSV. Third-party baseline checkpoints
or implementations are not redistributed when their licenses do not allow it;
for audit-level reproduction, release the raw route logs and the manifest used
by `tools/make_table4.py`.

**中文。** 我们新增了 `tools/compute_hidrive_ability_scores.py`、
`tools/make_table4.py` 和 `docs/HiLevAD/REPRODUCE_TABLE4.md`。给定每个方法的
逐路线日志（`route_results/route_*.json`），这些脚本可以重新计算 30 个
ability 分数并生成 Table 4 格式的 CSV。对于第三方 baseline，如果其 checkpoint
或实现许可证不允许再分发，仓库不会直接包含这些文件；audit-level 复现应发布
raw route logs 和 `tools/make_table4.py` 使用的 manifest。

## Quick check / 快速检查

```bash
cd /path/to/HiDrive
conda env create -f environment.yml
conda activate hilevad

export CARLA_ROOT=/path/to/carla_hilevad
bash scripts/prepare_carla_package.sh "$CARLA_ROOT"
pip install "$CARLA_ROOT"/PythonAPI/carla/dist/carla-*.whl
python -c "import carla; print(carla.__file__)"
```
