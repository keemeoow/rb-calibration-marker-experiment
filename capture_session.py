"""Safe sequential session allocation for calibration capture runs."""

from __future__ import annotations

import json
import os
import re
from dataclasses import dataclass
from datetime import datetime, timezone


SESSION_PREFIX = "session"
SESSION_DIGITS = 2
CALIBRATION_SUBDIR = "calib_train"
SESSION_SUBDIRS = (
    CALIBRATION_SUBDIR,
    "blind_test",
    "calib_out",
    "calibration_methods",
    "predictions",
    "audit",
)


@dataclass(frozen=True)
class CaptureSession:
    index: int
    session_id: str
    session_root: str
    capture_root: str
    manifest_path: str


def _existing_indices(data_root: str) -> list[int]:
    pattern = re.compile(rf"^{re.escape(SESSION_PREFIX)}([0-9]+)$")
    indices: list[int] = []
    try:
        entries = os.scandir(data_root)
    except FileNotFoundError:
        return indices
    with entries:
        for entry in entries:
            if not entry.is_dir(follow_symlinks=False):
                continue
            match = pattern.fullmatch(entry.name)
            if match:
                indices.append(int(match.group(1)))
    return indices


def allocate_next_capture_session(data_root: str = "data") -> CaptureSession:
    """Atomically reserve ``sessionNN`` and create its calibration capture root.

    Numbering always advances from the largest existing numbered session.  A
    directory is never reused, even if it is empty, so an interrupted or
    partially captured session cannot be overwritten silently.
    """
    data_root = os.path.abspath(os.path.expanduser(data_root))
    os.makedirs(data_root, exist_ok=True)
    next_index = max(_existing_indices(data_root), default=0) + 1

    while True:
        session_id = f"{SESSION_PREFIX}{next_index:0{SESSION_DIGITS}d}"
        session_root = os.path.join(data_root, session_id)
        try:
            os.mkdir(session_root)
            break
        except FileExistsError:
            next_index += 1

    for subdir in SESSION_SUBDIRS:
        os.mkdir(os.path.join(session_root, subdir))
    capture_root = os.path.join(session_root, CALIBRATION_SUBDIR)
    manifest_path = os.path.join(session_root, "session_manifest.json")
    manifest = {
        "artifact_schema": "capture_session_manifest_v1",
        "session_id": session_id,
        "session_index": int(next_index),
        "session_root": session_root,
        "calibration_capture_root": capture_root,
        "blind_test_root": os.path.join(session_root, "blind_test"),
        "allocated_at_utc": datetime.now(timezone.utc).isoformat(),
        "allocation_policy": "max_existing_index_plus_one_no_reuse",
        "status": "allocated",
    }
    with open(manifest_path, "x", encoding="utf-8") as handle:
        json.dump(manifest, handle, indent=2)
        handle.write("\n")

    return CaptureSession(
        index=next_index,
        session_id=session_id,
        session_root=session_root,
        capture_root=capture_root,
        manifest_path=manifest_path,
    )
