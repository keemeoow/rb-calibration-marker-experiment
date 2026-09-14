"""Safe sequential session allocation for calibration capture runs."""

from __future__ import annotations

import json
import os
from dataclasses import dataclass
from datetime import date, datetime, timezone

from capture_pipeline.paths import (
    REPO_ROOT,
    SESSION_DATE_FORMAT,
    SESSION_DIGITS,
    SESSION_PREFIX,
    require_data_path_inside,
    session_folder_name,
    session_index,
)

ZEUS_CALIBRATION_ROOT = REPO_ROOT / "zeus_gello_calibration"

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


def _date_suffix() -> str:
    """Return today's ``MMDD`` stamp used as the session folder suffix."""
    return date.today().strftime(SESSION_DATE_FORMAT)


def _existing_indices(data_root: str) -> list[int]:
    # 날짜/설명이 붙기 전에 만들어진 sessionNN 도 세어야 번호가 뒤로 가지 않는다.
    indices: list[int] = []
    try:
        entries = os.scandir(data_root)
    except FileNotFoundError:
        return indices
    with entries:
        for entry in entries:
            if not entry.is_dir(follow_symlinks=False):
                continue
            index = session_index(entry.name)
            if index is not None:
                indices.append(index)
    return indices


def allocate_next_capture_session(data_root: str,
                                  label: str | None = None) -> CaptureSession:
    """Reserve a numbered session under the explicit Zeus ``--data_root``.

    Numbering always advances from the largest existing numbered session.  A
    directory is never reused, even if it is empty, so an interrupted or
    partially captured session cannot be overwritten silently.  ``MMDD`` is the
    capture date, so the folder name alone says which day the data came from,
    and ``label`` (e.g. ``"zeus wrist motion"``) says what was captured.
    """
    data_root = str(require_data_path_inside(
        data_root, ZEUS_CALIBRATION_ROOT, label="Zeus data root"))
    os.makedirs(data_root, exist_ok=True)
    next_index = max(_existing_indices(data_root), default=0) + 1
    date_suffix = _date_suffix()

    while True:
        session_id = session_folder_name(next_index, label)
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
        "capture_date_suffix": date_suffix,
        "capture_label": label,
        "allocation_policy": "max_existing_index_plus_one_no_reuse",
        "naming_policy": "<--data_root>/session<NN>[_<label>]_<MMDD>",
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
