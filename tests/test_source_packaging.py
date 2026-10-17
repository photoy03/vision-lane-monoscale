"""Packaging tools are tested before use; no ROS/hardware required."""
import importlib.util
import json
from pathlib import Path
import sys
import tarfile

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools/build'))
import check_package
import export_source_package


def test_assets_and_packages():
    result = check_package.check_source(ROOT)
    assert result['errors'] == []
    assert len(result['packages']) == 11


def test_reject_path_escape(tmp_path):
    with pytest.raises(ValueError):
        check_package.checked_path(tmp_path, '../outside')


def test_export_and_integrity(tmp_path):
    output = tmp_path / 'source.tar.gz'
    result = export_source_package.export(ROOT, output)
    assert result['source_files'] > 100
    destination = tmp_path / 'extracted'
    with tarfile.open(output) as archive:
        members = archive.getmembers()
        for member in members:
            assert not member.issym() and not member.islnk()
            assert not Path(member.name).is_absolute() and '..' not in Path(member.name).parts
            assert not set(Path(member.name).parts) & export_source_package.SKIP_PARTS
        archive.extractall(destination)
    extracted = destination / 'vision_support_ws'
    assert check_package.check_source(extracted)['errors'] == []
    # Existing archives cannot be overwritten.
    with pytest.raises(FileExistsError):
        export_source_package.export(ROOT, output)
    (extracted / 'README.md').write_text('tampered')
    assert any('README.md' in error for error in check_package.check_source(extracted)['errors'])


def test_missing_and_modified_calibration(tmp_path):
    import shutil
    shutil.copytree(ROOT / 'tools/build/dependencies', tmp_path / 'tools/build/dependencies')
    shutil.copytree(ROOT / 'src', tmp_path / 'src')
    calibration = tmp_path / 'src/vision_mapping_bringup/config/calibration/front.npz'
    calibration.write_bytes(b'invalid NPZ')
    result = check_package.check_source(tmp_path)
    assert any('Changed asset' in error for error in result['errors'])
    calibration.unlink()
    assert any('Missing asset' in error for error in check_package.check_source(tmp_path)['errors'])
