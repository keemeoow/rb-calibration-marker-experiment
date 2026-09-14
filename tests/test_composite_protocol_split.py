from __future__ import annotations

import copy

import pytest

from calibration_pipeline.table1 import (
    COMPOSITE_PLACEMENT_SPLIT,
    build_composite_placement_split,
)
from capture_pipeline.waypoint_safety import (
    PHASE_P1,
    PHASE_P2,
    PHASE_P3,
    P3_STATIONARY_SET_INDEX,
    PROTOCOL_COMPOSITE_RIG_45,
)


def _capture(event_id: int, planned_id: str, phase: str, *,
             set_index=None, placement_id=None, view_index=None,
             selected=True) -> dict:
    payload = {
        "event_id": event_id,
        "planned_event_id": planned_id,
        "protocol_version": PROTOCOL_COMPOSITE_RIG_45,
        "phase": phase,
        "placement_id": placement_id,
        "view_index": view_index,
        "selected_for_analysis": selected,
    }
    if set_index is not None:
        payload["set_index"] = set_index
    return payload


def _meta() -> dict:
    captures = [
        _capture(event, f"P1_{event:02d}", PHASE_P1, view_index=event)
        for event in range(15)
    ]
    event = 15
    for placement in range(10):
        for view in range(2):
            captures.append(_capture(
                event,
                f"P2_PLACEMENT_{placement:02d}_VIEW_{view}",
                PHASE_P2,
                set_index=placement,
                placement_id=f"PLACEMENT_{placement:02d}",
                view_index=view,
            ))
            event += 1
    captures.extend([
        _capture(
            event + index,
            f"P3_{index:02d}",
            PHASE_P3,
            set_index=P3_STATIONARY_SET_INDEX,
            placement_id="PLACEMENT_09",
            view_index=index,
        )
        for index in range(10)
    ])
    return {
        "capture_config": {"capture_protocol": PROTOCOL_COMPOSITE_RIG_45},
        "captures": captures,
    }


def test_composite_split_keeps_both_views_of_each_placement_together() -> None:
    meta = _meta()
    split = build_composite_placement_split(meta, fraction=0.2, seed=20260731)

    assert split["strategy"] == COMPOSITE_PLACEMENT_SPLIT
    assert len(split["train_placement_ids"]) == 8
    assert len(split["heldout_placement_ids"]) == 2
    train = set(split["train_events"])
    heldout = set(split["test_events"])
    assert train.isdisjoint(heldout)
    assert set(range(15)) <= train
    assert set(range(35, 45)) <= train
    for detail in split["per_set"].values():
        events = set(detail["events"])
        assert events <= (heldout if detail["split_role"] == "heldout" else train)


def test_composite_split_ignores_unselected_retry_attempt() -> None:
    meta = _meta()
    retry = copy.deepcopy(meta["captures"][15])
    retry["event_id"] = 99
    retry["selected_for_analysis"] = False
    meta["captures"].append(retry)

    split = build_composite_placement_split(meta, fraction=0.2, seed=20260731)

    assert 99 not in split["train_events"]
    assert 99 not in split["test_events"]


def test_composite_split_rejects_p3_reusing_placement_09_set() -> None:
    meta = _meta()
    for capture in meta["captures"]:
        if capture["phase"] == PHASE_P3:
            capture["set_index"] = 9

    with pytest.raises(RuntimeError, match="P3 must use independent set_index"):
        build_composite_placement_split(meta, fraction=0.2, seed=20260731)
