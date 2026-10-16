#!/usr/bin/env bash
set -eo pipefail
support_tools=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
exec bash "$support_tools/build.sh" cuda
