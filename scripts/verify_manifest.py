#!/usr/bin/env python3
"""Cross-platform SHA256 manifest verifier."""

from __future__ import annotations

import argparse
import hashlib
from pathlib import Path


def digest(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("manifest", type=Path)
    args = parser.parse_args()
    manifest = args.manifest.expanduser().resolve()
    failures = []
    rows = 0
    for raw in manifest.read_text(encoding="utf-8-sig").splitlines():
        line = raw.strip()
        if not line:
            continue
        expected, relative = line.split(maxsplit=1)
        relative = relative.lstrip("* ")
        path = manifest.parent / relative
        rows += 1
        if not path.is_file():
            failures.append(f"MISSING {relative}")
        else:
            actual = digest(path)
            if actual.lower() != expected.lower():
                failures.append(f"MISMATCH {relative} expected={expected} actual={actual}")
    for failure in failures:
        print(failure)
    print(f"MANIFEST_{'FAIL' if failures else 'OK'} rows={rows} failures={len(failures)}")
    return 2 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
