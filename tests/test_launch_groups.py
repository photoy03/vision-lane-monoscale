"""Isolate launch stubs in a child process."""
from pathlib import Path
import subprocess
import sys


def test_functional_launch_groups(tmp_path):
    root = Path(__file__).resolve().parents[1]
    subprocess.run([sys.executable, str(root / 'tools/eval/checks/verify_path_equivalence.py'),
                    '--output', str(tmp_path / 'groups.json')], check=True)
