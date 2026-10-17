#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
test_dir=$(mktemp -d)
trap 'rm -rf "$test_dir"' EXIT
g++ -std=c++17 -O2 -Wall -Wextra -pedantic -I"$root/src/lane_mapping/lane_map/include" "$root/src/lane_mapping/lane_map/test/test_registration.cpp" -o "$test_dir/test"
"$test_dir/test"
