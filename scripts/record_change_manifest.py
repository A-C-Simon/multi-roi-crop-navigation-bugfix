"""Record the approved bug-fix changes relative to the source snapshot.

docs/source-snapshot.json is the historical import record and is never
rewritten. This script writes audit/implementation/change-manifest.json,
listing every snapshot file whose content was deliberately changed in this
bug-fix variant (with its baseline and current SHA-256) and every file added
by the implementation stage. scripts/verify_snapshot.py accepts a snapshot
file only if it is unchanged or matches this manifest exactly.

Run after an approved change, then review the manifest diff:

    python scripts/record_change_manifest.py
"""
from pathlib import Path
import hashlib
import json
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
MANIFEST = ROOT / "audit/implementation/change-manifest.json"
BASELINE_COMMIT = "d5c68594e8e633b4569a4a59f8c60f33ac785cf0"
ADDED = [
    "tests/unit/conftest.py",
    "tests/unit/synth.py",
    "tests/unit/test_geometry.py",
    "tests/unit/test_smoothing.py",
    "tests/unit/test_provenance.py",
    "tests/unit/test_recovery.py",
    "tests/unit/test_map.py",
    "tests/unit/test_runtime_adapters.py",
    "tests/unit/test_independent_review.py",
    "scripts/record_change_manifest.py",
]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    snapshot = json.loads((ROOT / "docs/source-snapshot.json").read_text())
    changed = {}
    for name, expected in sorted(snapshot["sha256"].items()):
        path = ROOT / name
        if not path.is_file():
            print(f"missing snapshot file: {name}", file=sys.stderr)
            return 1
        current = sha(path)
        if current != expected:
            changed[name] = {"baseline_sha256": expected, "current_sha256": current}
    added = {name: sha(ROOT / name) for name in ADDED if (ROOT / name).is_file()}
    # also report which of the 46 audited runtime/configuration files
    # (audit/baseline-files.json, official baseline hashes) now differ
    audited = json.loads((ROOT / "audit/baseline-files.json").read_text())["sha256"]
    audited_changed = sorted(n for n, h in audited.items()
                             if not (ROOT / n).is_file() or sha(ROOT / n) != h)
    try:
        head = subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True,
                              capture_output=True, check=True).stdout.strip()
    except Exception:
        head = ""
    record = {
        "description": "Approved bug-fix variant of the snapshot import. "
                       "See audit/implementation/REPORT.md for root causes.",
        "baseline_commit": BASELINE_COMMIT,
        "snapshot_record": "docs/source-snapshot.json",
        "recorded_at_head": head,
        "changed": changed,
        "added": added,
        "audited_baseline_files_changed": audited_changed,
    }
    MANIFEST.parent.mkdir(parents=True, exist_ok=True)
    MANIFEST.write_text(json.dumps(record, indent=2) + "\n")
    print(f"Recorded {len(changed)} changed snapshot files and {len(added)} added files")
    return 0


if __name__ == "__main__":
    sys.exit(main())
