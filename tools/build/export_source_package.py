#!/usr/bin/env python3
"""Export local source/assets, not system binaries, recordings or credentials."""
import argparse
import hashlib
import io
import json
from pathlib import Path
import tarfile

from check_package import check_source

ROOT = Path(__file__).resolve().parents[2]
TREES = ('src', 'tools', 'tests', 'third_party', 'docs')
FILES = ('README.md', '.gitignore')
SKIP_PARTS = {'__pycache__', '.pytest_cache', '.git'}
# colcon outputs; tools/build/ is source and is kept.
GENERATED_PARTS = {'build', 'install', 'log'}


def source_files(root):
    selected = set()
    for tree in TREES:
        for path in (root / tree).rglob('*'):
            rel = path.relative_to(root)
            generated = rel.parts[:2] != ('tools', 'build') and any(part in GENERATED_PARTS for part in rel.parts)
            if generated or any(part in SKIP_PARTS for part in rel.parts) or path.suffix in ('.pyc', '.orig', '.bag', '.db3', '.mcap'):
                continue
            if path.is_symlink():
                raise ValueError(f'Symlink not allowed in source export: {rel}')
            if path.is_file():
                selected.add(rel)
    for name in FILES:
        if (root / name).is_file():
            selected.add(Path(name))
    return sorted(selected)


def export(root, output):
    check = check_source(root)
    if check['errors']:
        raise ValueError(check['errors'])
    files = source_files(root)
    manifest = {'schema_version': 1, 'files': {
        str(rel): hashlib.sha256((root / rel).read_bytes()).hexdigest() for rel in files}}
    output.parent.mkdir(parents=True, exist_ok=True)
    # Exclusive creation: never overwrite an existing archive or source file.
    with output.open('xb') as stream, tarfile.open(fileobj=stream, mode='w:gz') as archive:
        for rel in files:
            archive.add(root / rel, arcname=str(Path('vision_support_ws') / rel), recursive=False)
        data = json.dumps(manifest, indent=2, ensure_ascii=False).encode('utf-8')
        item = tarfile.TarInfo('vision_support_ws/SOURCE_MANIFEST.json')
        item.size = len(data)
        item.mode = 0o644
        archive.addfile(item, io.BytesIO(data))
    return {'archive': str(output), 'source_files': len(files), 'bytes': output.stat().st_size,
            'sha256': hashlib.sha256(output.read_bytes()).hexdigest()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    print(json.dumps(export(ROOT, args.output.resolve()), indent=2))


if __name__ == '__main__':
    main()
