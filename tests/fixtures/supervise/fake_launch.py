#!/usr/bin/env python3
"""Stand-in for `ros2 launch` in tests/test_supervise_launch.py: the n-th start
does what the n-th line of $FAKE_PLAN says, and logs the start to $FAKE_LOG."""
import importlib.util, json, os, sys, time
from pathlib import Path

root = Path(__file__).resolve().parents[3]
spec = importlib.util.spec_from_file_location('clock_guard', root / 'src/vision_mapping_bringup/scripts/clock_guard.py')
log = Path(os.environ['FAKE_LOG'])
starts = log.read_text().splitlines() if log.exists() else []
log.write_text('\n'.join(starts + [os.environ['VISION_RUN_ID']]) + '\n')
if os.environ.get('FAKE_ENV_LOG'):  # what the supervisor told this start
    with open(os.environ['FAKE_ENV_LOG'], 'a') as out:
        out.write(json.dumps({'index': os.environ.get('VISION_RESTART_INDEX'),
                              'reason': os.environ.get('VISION_RESTART_REASON')}) + '\n')
plan = Path(os.environ['FAKE_PLAN']).read_text().split()
action = plan[min(len(starts), len(plan) - 1)]
if action == 'restart':      # what clock_guard leaves behind
    clock_guard = importlib.util.module_from_spec(spec)
    try:
        spec.loader.exec_module(clock_guard)
    except ImportError:      # no ROS Python here: same protocol, written directly
        clock_guard = None
    if clock_guard:
        clock_guard.request_restart('test')
    else:
        target = Path(os.environ['VISION_RESTART_REQUEST_DIR']) / (os.environ['VISION_RUN_ID'] + '.restart')
        target.write_text(json.dumps({'run_id': os.environ['VISION_RUN_ID'], 'reason': 'test'}))
elif action == 'stale':      # a request naming another run
    target = Path(os.environ['VISION_RESTART_REQUEST_DIR']) / (os.environ['VISION_RUN_ID'] + '.restart')
    target.write_text(json.dumps({'run_id': 'run_old', 'reason': 'test'}))
elif action == 'hang':
    time.sleep(60)
sys.exit(0)                  # ros2 launch exits 0 after a guard shutdown
