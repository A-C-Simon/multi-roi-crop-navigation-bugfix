"""Verify preserved source and asset files against their import checksums.

docs/source-snapshot.json records the original import and stays unchanged.
In this bug-fix variant, a snapshot file may differ from its import checksum
only when audit/implementation/change-manifest.json lists that exact change
(baseline hash equal to the snapshot record and current hash equal to the
file). Files added by the implementation stage are checked the same way.
Use --strict to require the unmodified import (the historical check).
"""
from pathlib import Path
import argparse
import hashlib
import json
import sys


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--strict", action="store_true",
                        help="ignore the change manifest; require the original import")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    record = json.loads((root / 'docs/source-snapshot.json').read_text())
    manifest_path = root / 'audit/implementation/change-manifest.json'
    manifest = {"changed": {}, "added": {}}
    if not args.strict and manifest_path.is_file():
        manifest = json.loads(manifest_path.read_text())
    mismatches, unchanged, approved = [], 0, []
    for name, expected in record['sha256'].items():
        path = root / name
        if not path.is_file():
            mismatches.append(f"{name} (missing)")
            continue
        current = sha(path)
        if current == expected:
            unchanged += 1
            continue
        entry = manifest.get("changed", {}).get(name)
        if entry and entry.get("baseline_sha256") == expected \
                and entry.get("current_sha256") == current:
            approved.append(name)
        else:
            mismatches.append(name)
    for name, expected in manifest.get("added", {}).items():
        path = root / name
        if not path.is_file() or sha(path) != expected:
            mismatches.append(f"{name} (added file differs from manifest)")
    if mismatches:
        print('Snapshot differences not covered by the change manifest:',
              *mismatches, sep='\n  ')
        return 1
    print(f"Verified {unchanged} unchanged source and asset files")
    if approved or manifest.get("added"):
        print(f"Verified {len(approved)} approved bug-fix changes and "
              f"{len(manifest.get('added', {}))} added files against "
              f"{manifest_path.relative_to(root)}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
