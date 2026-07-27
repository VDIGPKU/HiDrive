#!/usr/bin/env bash
set -euo pipefail

CARLA_HOME="${1:-${CARLA_ROOT:-${CARLA_PACKAGE_ROOT:-}}}"
if [ -z "${CARLA_HOME}" ]; then
  echo "usage: $0 /path/to/carla_hilevad" >&2
  echo "or set CARLA_ROOT=/path/to/carla_hilevad" >&2
  exit 2
fi

if [ ! -d "${CARLA_HOME}" ]; then
  echo "error: CARLA package directory not found: ${CARLA_HOME}" >&2
  exit 1
fi

linux_root="${CARLA_HOME}/Linux"
if [ -d "${linux_root}" ]; then
  launch_root="${linux_root}"
else
  launch_root="${CARLA_HOME}"
fi

if [ -x "${launch_root}/CarlaUnreal.sh" ] && [ ! -e "${launch_root}/CarlaUE4.sh" ]; then
  ln -s CarlaUnreal.sh "${launch_root}/CarlaUE4.sh"
  echo "created ${launch_root}/CarlaUE4.sh -> CarlaUnreal.sh"
elif [ -x "${launch_root}/CarlaUE4.sh" ]; then
  echo "launcher ok: ${launch_root}/CarlaUE4.sh"
elif [ -x "${launch_root}/CarlaUnreal.sh" ]; then
  echo "launcher ok: ${launch_root}/CarlaUnreal.sh"
else
  echo "error: no CARLA launcher found under ${launch_root}" >&2
  echo "expected CarlaUE4.sh or CarlaUnreal.sh" >&2
  exit 1
fi

lib_root="${launch_root}/CarlaUnreal/Plugins/Carla/Binaries/Linux"
if [ -d "${lib_root}" ]; then
  if [ -f "${lib_root}/libfoonathan_memory-0.7.3.so" ] && [ ! -e "${lib_root}/libfoonathan_memory-0.7.4.so" ]; then
    ln -s libfoonathan_memory-0.7.3.so "${lib_root}/libfoonathan_memory-0.7.4.so"
    echo "created ${lib_root}/libfoonathan_memory-0.7.4.so -> libfoonathan_memory-0.7.3.so"
  elif [ -e "${lib_root}/libfoonathan_memory-0.7.4.so" ]; then
    echo "libfoonathan compatibility ok: ${lib_root}/libfoonathan_memory-0.7.4.so"
  else
    echo "warning: libfoonathan_memory-0.7.3.so not found under ${lib_root}" >&2
  fi
else
  echo "warning: CARLA plugin library directory not found: ${lib_root}" >&2
fi

wheel_count=$(find "${CARLA_HOME}/PythonAPI/carla/dist" -maxdepth 1 -type f \( -name 'carla-*.whl' -o -name 'carla-*.egg' \) 2>/dev/null | wc -l)
if [ "${wheel_count}" -eq 0 ]; then
  echo "warning: no CARLA PythonAPI wheel/egg found under ${CARLA_HOME}/PythonAPI/carla/dist" >&2
else
  echo "CARLA PythonAPI packages:"
  find "${CARLA_HOME}/PythonAPI/carla/dist" -maxdepth 1 -type f \( -name 'carla-*.whl' -o -name 'carla-*.egg' \) | sort
fi

echo "CARLA package preparation complete."
