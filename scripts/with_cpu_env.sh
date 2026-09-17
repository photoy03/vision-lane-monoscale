#!/usr/bin/env bash
set -eo pipefail
stage1_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source "$stage1_root/scripts/cpu_environment.bash"
if [[ ! -f "$stage1_root/install/setup.bash" ]]; then
  echo 'Build this workspace first: bash scripts/build_cpu.sh' >&2
  exit 1
fi
source "$stage1_root/install/setup.bash"
if [[ $# -eq 0 ]]; then
  echo 'Usage: bash scripts/with_cpu_env.sh ros2 <command> ...' >&2
  exit 2
fi
exec "$@"
