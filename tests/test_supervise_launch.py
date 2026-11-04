"""tools/run/supervise_launch.sh restarts only on this run's own restart request."""
import json
import os
import signal
import subprocess
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / 'tools/run/supervise_launch.sh'
FAKE = ROOT / 'tests/fixtures/supervise/fake_launch.py'


def start(tmp_path, plan, **env):
    (tmp_path / 'plan').write_text(' '.join(plan))
    environment = dict(os.environ, FAKE_PLAN=str(tmp_path / 'plan'), FAKE_LOG=str(tmp_path / 'starts'),
                       VISION_SUPERVISE_COMMAND=f'/usr/bin/python3 {FAKE}', TMPDIR=str(tmp_path), **env)
    return subprocess.Popen(['bash', str(SCRIPT)], env=environment, stderr=subprocess.PIPE, text=True,
                            start_new_session=True)


def starts(tmp_path):
    path = tmp_path / 'starts'
    return path.read_text().split() if path.exists() else []


def test_restarts_on_own_request_and_stops_after_a_plain_exit(tmp_path):
    p = start(tmp_path, ['restart', 'crash'])
    _, err = p.communicate(timeout=30)
    assert p.returncode == 1
    ids = starts(tmp_path)
    assert len(ids) == 2 and len(set(ids)) == 2      # a new run ID for every start
    assert 'not restarting' in err


def test_request_of_another_run_is_ignored(tmp_path):
    p = start(tmp_path, ['stale'])
    p.communicate(timeout=30)
    assert p.returncode == 1 and len(starts(tmp_path)) == 1


def test_restart_limit(tmp_path):
    p = start(tmp_path, ['restart'], VISION_SUPERVISE_MAX_RESTARTS='2')
    _, err = p.communicate(timeout=60)
    assert p.returncode == 2 and len(starts(tmp_path)) == 3 and 'giving up' in err


def test_ctrl_c_during_the_wait_does_not_start_again(tmp_path):
    p = start(tmp_path, ['restart', 'hang'])
    deadline = time.time() + 10
    while len(starts(tmp_path)) < 1 and time.time() < deadline:
        time.sleep(0.05)
    time.sleep(1.0)                       # inside the 3 s wait
    os.killpg(p.pid, signal.SIGINT)
    p.communicate(timeout=30)
    assert p.returncode == 0 and len(starts(tmp_path)) == 1


def test_ctrl_c_while_running(tmp_path):
    p = start(tmp_path, ['hang'])
    deadline = time.time() + 10
    while len(starts(tmp_path)) < 1 and time.time() < deadline:
        time.sleep(0.05)
    os.killpg(p.pid, signal.SIGINT)
    p.communicate(timeout=30)
    assert p.returncode == 0 and len(starts(tmp_path)) == 1


def test_restart_index_and_reason_are_passed_to_each_start(tmp_path):
    p = start(tmp_path, ['restart', 'restart', 'crash'], FAKE_ENV_LOG=str(tmp_path / 'env'))
    p.communicate(timeout=60)
    seen = [json.loads(line) for line in (tmp_path / 'env').read_text().splitlines()]
    assert [s['index'] for s in seen] == ['0', '1', '2']
    assert seen[0]['reason'] == ''
    assert all('"reason"' in s['reason'] and '\n' not in s['reason'] for s in seen[1:])

