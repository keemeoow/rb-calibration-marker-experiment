#!/usr/bin/env python3
"""Export cube observation quality as a readable HTML report and raw data."""

from __future__ import annotations

import argparse
import csv
import html
import json
import os
import sys
from collections import defaultdict
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from calibration_pipeline.apriltag_cube import AprilTagCubeTarget  # noqa: E402
from calibration_pipeline.cube_config import load_cube_config_from_meta  # noqa: E402
from calibration_pipeline.observations import load_cube_pixel_observations  # noqa: E402
from calibration_pipeline.runtime import load_intrinsics_with_depth_scale  # noqa: E402


CAMERA_FIELDS = (
    "event_id", "camera_id", "set_idx", "is_gripper_camera",
    "detected_marker_ids", "marker_ids", "observed_faces",
    "observed_face_count", "noncoplanar_face_count", "is_planar",
    "quality_tier", "pnp_rmse_px", "pnp_inlier_fraction", "pnp_solver",
    "positive_depth_candidate_count", "pnp_candidate_count", "pnp_accepted",
    "detection_method", "redetection_attempted", "redetection_selected",
    "recovered_core_observation", "redetection_max_rmse_px",
    "detection_candidate_count",
    "baseline_detected_marker_ids", "baseline_marker_ids",
    "baseline_quality_tier", "baseline_pnp_rmse_px",
    "legacy_policy_selected", "core_multiface_selected",
    "selected_for_calibration", "selection_reason", "status",
)

EVENT_FIELDS = (
    "event_id", "camera_ids", "pnp_accepted_camera_ids",
    "core_multiface_camera_ids", "planar_multimarker_camera_ids",
    "single_marker_camera_ids", "observed_marker_ids", "observed_faces",
    "has_core_multiface_observation",
)


def _csv_value(value):
    if isinstance(value, (list, tuple)):
        return ";".join(str(item) for item in value)
    return value


def _write_csv(path, rows, fields):
    with open(path, "w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: _csv_value(row.get(field)) for field in fields})


def _join(values, empty="—"):
    return ", ".join(str(value) for value in (values or [])) or empty


def _badge(label, kind):
    return f'<span class="badge {html.escape(kind)}">{html.escape(str(label))}</span>'


def _write_html(path, report):
    diagnostics = report["diagnostics"]
    records = diagnostics["observation_quality_by_event_camera"]
    events = diagnostics["event_quality_summary"]
    comparison = diagnostics["available_observation_policy_comparison"]
    grouped = defaultdict(list)
    for record in records:
        grouped[int(record["event_id"])].append(record)

    core_event_count = sum(
        bool(event["has_core_multiface_observation"]) for event in events)
    rmse = diagnostics["accepted_pnp_rmse_px"]
    event_sections = []
    for event in events:
        event_id = int(event["event_id"])
        event_records = sorted(grouped[event_id], key=lambda item: int(item["camera_id"]))
        set_idx = next(
            (record.get("set_idx") for record in event_records
             if record.get("set_idx") is not None),
            None,
        )
        has_core = bool(event["has_core_multiface_observation"])
        has_single = bool(event["single_marker_camera_ids"])
        has_planar = bool(event["planar_multimarker_camera_ids"])
        category = "core" if has_core else ("planar" if has_planar else (
            "single" if has_single else "none"))
        summary_badge = _badge(
            "CORE" if has_core else ("PLANAR" if has_planar else (
                "SINGLE" if has_single else "NO OBS")),
            category,
        )

        camera_rows = []
        for record in event_records:
            tier = str(record.get("quality_tier", "none"))
            tier_kind = {
                "nonplanar_multiface": "core",
                "planar_multimarker": "planar",
                "single_marker": "single",
            }.get(tier, "none")
            rmse_value = record.get("pnp_rmse_px")
            rmse_text = "—" if rmse_value is None else f"{float(rmse_value):.3f} px"
            face_count = int(record.get("observed_face_count", 0))
            support = (
                f"{face_count} {'face' if face_count == 1 else 'faces'} / "
                f"{int(record.get('noncoplanar_face_count', 0))} non-coplanar"
            )
            if record.get("is_planar") is True:
                support += " / planar"
            elif record.get("is_planar") is False:
                support += " / non-planar"
            candidates = (
                f"{int(record.get('positive_depth_candidate_count', 0))} / "
                f"{int(record.get('pnp_candidate_count', 0))}"
            )
            selected = bool(record.get("selected_for_calibration"))
            camera_rows.append(f"""
                <tr class="{'selected' if selected else ''}">
                  <td><strong>cam{int(record['camera_id'])}</strong>{' <small>gripper</small>' if record.get('is_gripper_camera') else ''}</td>
                  <td>{html.escape(_join(record.get('marker_ids')))}</td>
                  <td>{html.escape(_join(record.get('observed_faces')))}</td>
                  <td>{html.escape(support)}</td>
                  <td>{html.escape(rmse_text)}</td>
                  <td>{html.escape(candidates)}</td>
                  <td>{_badge(tier.replace('_', ' '), tier_kind)}</td>
                  <td>{_badge('selected', 'core') if selected else html.escape(str(record.get('selection_reason', '—')))}</td>
                </tr>
            """)

        event_sections.append(f"""
          <details class="event" data-category="{category}" data-event="{event_id}">
            <summary>
              <span class="event-title">Event {event_id:02d}</span>
              <span class="set-label">set {html.escape(str(set_idx)) if set_idx is not None else '—'}</span>
              {summary_badge}
              <span class="summary-text">core cams: {html.escape(_join(event['core_multiface_camera_ids']))}</span>
              <span class="summary-text">IDs: {html.escape(_join(event['observed_marker_ids']))}</span>
              <span class="summary-text">faces: {html.escape(_join(event['observed_faces']))}</span>
            </summary>
            <div class="detail-wrap">
              <table>
                <thead><tr>
                  <th>Camera</th><th>Marker IDs</th><th>Faces</th><th>3D support</th>
                  <th>PnP RMSE</th><th>Positive / all</th><th>Quality tier</th><th>Calibration</th>
                </tr></thead>
                <tbody>{''.join(camera_rows)}</tbody>
              </table>
            </div>
          </details>
        """)

    document = f"""<!doctype html>
<html lang="ko">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Session04 Cube Observation Quality</title>
  <style>
    :root {{ color-scheme: light dark; --bg:#f5f7fa; --surface:#fff; --text:#18212b;
      --muted:#657180; --border:#d8dee7; --core:#16854b; --core-bg:#e7f7ee;
      --single:#a35c00; --single-bg:#fff2d9; --planar:#7048a8; --planar-bg:#f1eafb;
      --none:#687280; --none-bg:#edf0f3; --selected:#f1fbf5; }}
    @media (prefers-color-scheme: dark) {{ :root {{ --bg:#101419; --surface:#171c22;
      --text:#eef2f6; --muted:#a7b0bb; --border:#343c46; --core:#62d394;
      --core-bg:#173b29; --single:#ffc56d; --single-bg:#483418; --planar:#c5a0f4;
      --planar-bg:#38294d; --none:#bdc5cf; --none-bg:#2a3037; --selected:#162a20; }} }}
    * {{ box-sizing:border-box; }}
    body {{ margin:0; background:var(--bg); color:var(--text); font:14px/1.45 -apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif; }}
    main {{ max-width:1500px; margin:auto; padding:28px; }}
    h1 {{ margin:0 0 5px; font-size:26px; }}
    .subtitle {{ color:var(--muted); margin-bottom:22px; }}
    .stats {{ display:grid; grid-template-columns:repeat(6,minmax(120px,1fr)); gap:10px; margin-bottom:18px; }}
    .stat {{ background:var(--surface); border:1px solid var(--border); border-radius:10px; padding:13px 15px; }}
    .stat strong {{ display:block; font-size:22px; }} .stat span {{ color:var(--muted); font-size:12px; }}
    .contract {{ background:var(--surface); border:1px solid var(--border); border-radius:10px; padding:12px 15px; margin-bottom:14px; }}
    code {{ font-family:ui-monospace,SFMono-Regular,Menlo,monospace; font-size:12px; }}
    .toolbar {{ position:sticky; top:0; z-index:5; display:flex; flex-wrap:wrap; gap:8px; background:color-mix(in srgb,var(--bg) 92%,transparent); padding:10px 0; backdrop-filter:blur(7px); }}
    input,select,button {{ border:1px solid var(--border); background:var(--surface); color:var(--text); border-radius:7px; padding:8px 10px; font:inherit; }}
    input {{ min-width:220px; }} button {{ cursor:pointer; }}
    details.event {{ background:var(--surface); border:1px solid var(--border); border-radius:9px; margin:8px 0; overflow:hidden; }}
    summary {{ cursor:pointer; display:flex; align-items:center; flex-wrap:wrap; gap:10px; padding:12px 14px; list-style:none; }}
    summary::-webkit-details-marker {{ display:none; }} summary::before {{ content:'›'; font-size:21px; color:var(--muted); transition:.15s; }}
    details[open] summary::before {{ transform:rotate(90deg); }}
    .event-title {{ font-size:16px; font-weight:700; min-width:72px; }} .set-label,.summary-text {{ color:var(--muted); }}
    .badge {{ display:inline-block; border-radius:999px; padding:2px 8px; font-size:11px; font-weight:700; white-space:nowrap; }}
    .badge.core {{ color:var(--core); background:var(--core-bg); }} .badge.single {{ color:var(--single); background:var(--single-bg); }}
    .badge.planar {{ color:var(--planar); background:var(--planar-bg); }} .badge.none {{ color:var(--none); background:var(--none-bg); }}
    .detail-wrap {{ overflow:auto; border-top:1px solid var(--border); }}
    table {{ width:100%; border-collapse:collapse; min-width:970px; }} th,td {{ padding:9px 12px; text-align:left; border-bottom:1px solid var(--border); white-space:nowrap; }}
    th {{ color:var(--muted); font-size:11px; text-transform:uppercase; letter-spacing:.04em; }} tr.selected {{ background:var(--selected); }}
    td small {{ display:block; color:var(--muted); }} .hidden {{ display:none !important; }}
    .legend {{ color:var(--muted); margin:10px 0 4px; }}
    @media (max-width:900px) {{ main {{ padding:16px; }} .stats {{ grid-template-columns:repeat(2,1fr); }} .summary-text {{ display:none; }} }}
  </style>
</head>
<body><main>
  <h1>Session04 Cube Observation Quality</h1>
  <div class="subtitle">판정 단위: event × camera · policy: <strong>{html.escape(report['policy'])}</strong> · gripped events 제외</div>
  <section class="stats">
    <div class="stat"><strong>{len(events)}</strong><span>전체 events</span></div>
    <div class="stat"><strong>{core_event_count}</strong><span>core 관측 보유 events</span></div>
    <div class="stat"><strong>{comparison['core_multiface']}</strong><span>선택된 core 관측</span></div>
    <div class="stat"><strong>{comparison['legacy']}</strong><span>legacy 관측</span></div>
    <div class="stat"><strong>{comparison['single_marker']}</strong><span>single-marker</span></div>
    <div class="stat"><strong>{float(rmse['median']):.3f}px</strong><span>PnP RMSE median</span></div>
  </section>
  <div class="contract"><strong>Core 조건</strong> &nbsp;
    <code>face ≥ 2 · non-coplanar face ≥ 2 · non-planar · positive-depth candidate ≥ 1</code>
  </div>
  <div class="toolbar">
    <input id="event-search" type="search" placeholder="Event 번호 검색 (예: 12)">
    <select id="category-filter" aria-label="품질 필터">
      <option value="all">모든 event</option><option value="core">Core event</option>
      <option value="single">Single-marker only</option><option value="planar">Planar multi-tag only</option>
      <option value="none">유효 관측 없음</option>
    </select>
    <button id="expand-all" type="button">모두 펼치기</button>
    <button id="collapse-all" type="button">모두 접기</button>
    <span id="visible-count" class="summary-text"></span>
  </div>
  <div class="legend">Positive / all은 모든 3D corner가 카메라 앞쪽(z&gt;0)에 놓이는 PnP 후보 수 / 전체 후보 수입니다.</div>
  <section id="events">{''.join(event_sections)}</section>
</main>
<script>
(() => {{
  const events = [...document.querySelectorAll('details.event')];
  const search = document.getElementById('event-search');
  const filter = document.getElementById('category-filter');
  const count = document.getElementById('visible-count');
  function apply() {{
    const query = search.value.trim(); const category = filter.value; let visible = 0;
    for (const event of events) {{
      const numericQuery = Number(query);
      const matchesQuery = !query || (
        Number.isInteger(numericQuery) && Number(event.dataset.event) === numericQuery
      );
      const matchesCategory = category === 'all' || event.dataset.category === category;
      event.classList.toggle('hidden', !(matchesQuery && matchesCategory));
      if (matchesQuery && matchesCategory) visible += 1;
    }}
    count.textContent = `${{visible}} events 표시`;
  }}
  search.addEventListener('input', apply); filter.addEventListener('change', apply);
  document.getElementById('expand-all').addEventListener('click', () => events.filter(e => !e.classList.contains('hidden')).forEach(e => e.open = true));
  document.getElementById('collapse-all').addEventListener('click', () => events.forEach(e => e.open = false));
  apply();
}})();
</script></body></html>"""
    with open(path, "w", encoding="utf-8") as stream:
        stream.write(document)


def _md_cell(value):
    """Return a Markdown-table-safe string."""
    return str(value).replace("|", r"\|").replace("\n", "<br>")


def _write_markdown(path, report, meta):
    diagnostics = report["diagnostics"]
    records = diagnostics["observation_quality_by_event_camera"]
    events = diagnostics["event_quality_summary"]
    comparison = diagnostics["available_observation_policy_comparison"]
    grouped = defaultdict(list)
    for record in records:
        grouped[int(record["event_id"])].append(record)

    core_event_count = sum(
        bool(event["has_core_multiface_observation"]) for event in events)
    no_accepted_event_count = sum(
        not event["pnp_accepted_camera_ids"] for event in events)
    rmse = diagnostics["accepted_pnp_rmse_px"]
    session_root = Path(report["session_root"])
    session_name = session_root.parent.name
    nonselected = [
        record for record in records
        if not record.get("selected_for_calibration")
    ]
    no_marker = [
        record for record in nonselected
        if record.get("status") == "no_markers_detected"
    ]
    pnp_rejected = [
        record for record in nonselected
        if record.get("status") == "pnp_rmse_rejected"
    ]
    single_marker = [
        record for record in nonselected
        if record.get("pnp_accepted")
        and record.get("quality_tier") == "single_marker"
    ]
    high_rmse = sorted(
        (record for record in records
         if record.get("selected_for_calibration")
         and float(record.get("pnp_rmse_px") or 0.0) >= 2.0),
        key=lambda record: -float(record["pnp_rmse_px"]),
    )
    recovered_core_records = sorted(
        (record for record in records
         if record.get("recovered_core_observation")),
        key=lambda record: (
            int(record["event_id"]), int(record["camera_id"])),
    )

    def event_list(items):
        return ", ".join(
            f"E{int(record['event_id']):02d}" for record in items) or "—"

    def observation_list(items):
        return ", ".join(
            f"E{int(record['event_id']):02d}/cam{int(record['camera_id'])}"
            for record in items
        ) or "—"

    def best_recovery_rmse(record):
        values = [
            float(candidate["pnp_rmse_px"])
            for candidate in record.get("redetection_candidates", [])
            if candidate.get("method") != "default"
            and candidate.get("quality_tier") == "nonplanar_multiface"
            and candidate.get("pnp_rmse_px") is not None
        ]
        return min(values) if values else None

    capture_config = meta.get("capture_config", {})
    placement_gate = (
        capture_config.get("capture_gate", {})
        .get("profiles", {})
        .get("A_placement", {})
    )
    capture_by_event = {
        int(capture["event_id"]): capture
        for capture in meta.get("captures", [])
    }
    no_marker_charuco = []
    no_marker_fixed_visible = []
    for record in no_marker:
        gate = capture_by_event.get(int(record["event_id"]), {}).get(
            "capture_gate", {})
        value = gate.get("gripper_charuco_corners")
        if value is not None:
            no_marker_charuco.append(int(value))
        value = gate.get("fixed_visible_cams")
        if value is not None:
            no_marker_fixed_visible.append(int(value))

    lines = [
        f"# {session_name} Cube Observation Quality",
        "",
        f"- 판정 단위: `event × camera`",
        f"- 관측 선택 정책: `{report['policy']}`",
        f"- Gripped events: `{'제외' if report['exclude_gripped'] else '포함'}`",
        f"- Cube config: `{report['cube_config_source']}`",
        "",
        "## 전체 요약",
        "",
        "| 항목 | 결과 |",
        "|---|---:|",
        f"| 전체 events | {len(events)} |",
        f"| Core 관측 보유 events | {core_event_count} |",
        f"| 선택된 core 관측 | {comparison['core_multiface']} |",
        f"| 기본 검출 core 관측 | {comparison['core_multiface'] - len(recovered_core_records)} |",
        f"| 재검출로 복구한 core 관측 | {len(recovered_core_records)} |",
        f"| Legacy 기준 관측 | {comparison['legacy']} |",
        f"| Single-marker 관측 | {comparison['single_marker']} |",
        f"| Planar multi-tag 관측 | {comparison['planar_multimarker']} |",
        f"| PnP 미채택 events | {no_accepted_event_count} |",
        f"| PnP RMSE median | {float(rmse['median']):.3f} px |",
        f"| PnP RMSE range | {float(rmse['min']):.3f}–{float(rmse['max']):.3f} px |",
        "",
        "## Core 관측 조건",
        "",
        "Calibration 핵심 관측은 아래 조건을 모두 만족합니다.",
        "",
        "- 관측 face 수 ≥ 2",
        "- Non-coplanar face 수 ≥ 2",
        "- 3D corner 구성이 non-planar",
        "- Positive-depth PnP 후보 수 ≥ 1",
        "",
        "`Positive / all`은 모든 3D corner가 카메라 앞쪽(`z > 0`)에 놓이는 "
        "PnP 후보 수와 전체 후보 수입니다.",
        "",
        "## 원본 영상 재검출 결과",
        "",
        f"기본 검출이 core 조건을 통과하지 못한 "
        f"{diagnostics['counts'].get('redetection_attempts', 0)}개 관측에만 "
        "subpixel refinement, 완화된 AprilTag threshold, 2배 업스케일 및 "
        "unsharp 전처리를 적용했습니다. 복구 관측에는 더 엄격한 RMSE 3 px "
        "상한을 적용했습니다. 기존 core 관측은 다시 선택하지 않아 "
        "원래 corner를 그대로 보존합니다.",
        "",
        "| Event | 기본 결과 | 선택한 재검출 | 복구 IDs | Faces | 최종 RMSE |",
        "|---:|---|---|---|---|---:|",
        *[
            "| " + " | ".join(_md_cell(value) for value in (
                f"{int(record['event_id']):02d}/cam{int(record['camera_id'])}",
                f"{str(record.get('baseline_quality_tier', 'none')).replace('_', ' ')}"
                + (f" · {float(record['baseline_pnp_rmse_px']):.3f} px"
                   if record.get("baseline_pnp_rmse_px") is not None else ""),
                record.get("detection_method", "—"),
                _join(record.get("marker_ids")),
                _join(record.get("observed_faces")),
                f"{float(record['pnp_rmse_px']):.3f} px",
            )) + " |"
            for record in recovered_core_records
        ],
        "",
        "![재검출 복구 오버레이](redetection_recovered_core_overlay.png)",
        "",
        "오버레이에서 빨강(`B`)은 기본 검출, 초록(`R`)은 최종 재검출입니다.",
        "",
        "## 제외 및 저품질 원인",
        "",
        "| 구분 | 수 | Events | 직접 판정 원인 |",
        "|---|---:|---|---|",
        f"| Cube marker 미검출 | {len(no_marker)} | {event_list(no_marker)} | "
        "저장된 영상에서 설정된 cube marker가 검출되지 않음 |",
        f"| PnP RMSE 초과 | {len(pnp_rejected)} | {event_list(pnp_rejected)} | "
        "Multi-face corner는 검출됐지만 cam2 허용치 5 px 초과 |",
        f"| Single marker | {len(single_marker)} | {event_list(single_marker)} | "
        "한 면만 보여 planar pose 후보가 2개이므로 core 조건 미충족 |",
        f"| 선택됐지만 RMSE ≥ 2 px | {len(high_rmse)} | {observation_list(high_rmse)} | "
        "대부분 한 corner의 큰 residual이 전체 RMSE를 상승시킴 |",
        "",
        "### Capture 구조상 원인",
        "",
        f"- `a_fixed_cam_views_per_set = {capture_config.get('a_fixed_cam_views_per_set', '—')}`이므로 "
        "각 set의 첫 event 이후에는 고정 카메라 영상이 저장되지 않고 cam2만 남습니다.",
        f"- `A_placement.min_gripper_markers = {placement_gate.get('min_gripper_markers', '—')}`, "
        f"`require_gripper_cube_pnp = {str(placement_gate.get('require_gripper_cube_pnp', '—')).lower()}`, "
        f"`max_cube_pnp_reproj_mean_px = {placement_gate.get('max_cube_pnp_reproj_mean_px', '—')}`로 "
        "cube 관측 품질 게이트가 사실상 비활성화되어 있습니다.",
        f"- Cube marker 미검출 event에서도 cam2 ChArUco corner는 "
        f"{min(no_marker_charuco) if no_marker_charuco else '—'}–"
        f"{max(no_marker_charuco) if no_marker_charuco else '—'}개이고, "
        f"고정 카메라 cube 가시 수는 "
        f"{min(no_marker_fixed_visible) if no_marker_fixed_visible else '—'}–"
        f"{max(no_marker_fixed_visible) if no_marker_fixed_visible else '—'}대입니다. "
        "따라서 이는 전체 event 실패가 아니라, 저장된 cam2의 cube 관측 부재입니다.",
        "",
        "### PnP RMSE 초과 상세",
        "",
        "| Event | Camera | Marker IDs | Faces | 기본 RMSE | 재검출 최저 | 복구 상한 |",
        "|---:|---|---|---|---:|---:|---:|",
        *[
            "| " + " | ".join(_md_cell(value) for value in (
                f"{int(record['event_id']):02d}",
                f"cam{int(record['camera_id'])}",
                _join(record.get("marker_ids")),
                _join(record.get("observed_faces")),
                f"{float(record['pnp_rmse_px']):.3f} px",
                (f"{best_recovery_rmse(record):.3f} px"
                 if best_recovery_rmse(record) is not None else "—"),
                f"{float(record.get('redetection_max_rmse_px') or 3.0):.3f} px",
            )) + " |"
            for record in pnp_rejected
        ],
        "",
        "### 선택된 관측 중 RMSE ≥ 2 px",
        "",
        "| Event | Camera | Marker IDs | RMSE | Inlier fraction |",
        "|---:|---|---|---:|---:|",
        *[
            "| " + " | ".join(_md_cell(value) for value in (
                f"{int(record['event_id']):02d}",
                f"cam{int(record['camera_id'])}",
                _join(record.get("marker_ids")),
                f"{float(record['pnp_rmse_px']):.3f} px",
                f"{float(record['pnp_inlier_fraction']):.3f}",
            )) + " |"
            for record in high_rmse
        ],
        "",
        "## Event 요약",
        "",
        "| Event | Set | 결과 | PnP cameras | Core cameras | Marker IDs | Faces |",
        "|---:|---:|---|---|---|---|---|",
    ]

    event_details = []
    for event in events:
        event_id = int(event["event_id"])
        event_records = sorted(
            grouped[event_id], key=lambda item: int(item["camera_id"]))
        set_idx = next(
            (record.get("set_idx") for record in event_records
             if record.get("set_idx") is not None),
            None,
        )
        has_core = bool(event["has_core_multiface_observation"])
        has_planar = bool(event["planar_multimarker_camera_ids"])
        has_single = bool(event["single_marker_camera_ids"])
        event_marker_ids = sorted({
            int(marker_id) for record in event_records
            for marker_id in record.get("marker_ids", [])
        })
        event_faces = sorted({
            str(face) for record in event_records
            for face in record.get("observed_faces", [])
        })
        has_usable_markers = bool(event_marker_ids)
        result = (
            "CORE" if has_core else
            "PLANAR MULTI-TAG" if has_planar else
            "SINGLE MARKER" if has_single else
            "PNP REJECTED" if has_usable_markers else
            "NO MARKER"
        )
        summary_values = (
            f"{event_id:02d}",
            "—" if set_idx is None else set_idx,
            result,
            _join(event["pnp_accepted_camera_ids"]),
            _join(event["core_multiface_camera_ids"]),
            _join(event_marker_ids),
            _join(event_faces),
        )
        lines.append("| " + " | ".join(_md_cell(v) for v in summary_values) + " |")

        event_details.extend([
            "",
            f"<details><summary><strong>Event {event_id:02d}</strong> · "
            f"set {set_idx if set_idx is not None else '—'} · {result}</summary>",
            "",
            "| Camera | Detection | Detected IDs | Used IDs | Faces | Face support | Planarity | PnP RMSE | Positive / all | Quality | Calibration |",
            "|---|---|---|---|---|---|---|---:|---:|---|---|",
        ])
        for record in event_records:
            face_count = int(record.get("observed_face_count", 0))
            noncoplanar_count = int(record.get("noncoplanar_face_count", 0))
            if record.get("is_planar") is True:
                planarity = "planar"
            elif record.get("is_planar") is False:
                planarity = "non-planar"
            else:
                planarity = "—"
            rmse_value = record.get("pnp_rmse_px")
            rmse_text = "—" if rmse_value is None else f"{float(rmse_value):.3f} px"
            selected = bool(record.get("selected_for_calibration"))
            camera_label = f"cam{int(record['camera_id'])}"
            if record.get("is_gripper_camera"):
                camera_label += " (gripper)"
            detail_values = (
                camera_label,
                (f"{record.get('detection_method', 'default')} (recovered)"
                 if record.get("recovered_core_observation")
                 else record.get("detection_method", "default")),
                _join(record.get("detected_marker_ids")),
                _join(record.get("marker_ids")),
                _join(record.get("observed_faces")),
                f"{face_count} {'face' if face_count == 1 else 'faces'} / "
                f"{noncoplanar_count} non-coplanar",
                planarity,
                rmse_text,
                f"{int(record.get('positive_depth_candidate_count', 0))} / "
                f"{int(record.get('pnp_candidate_count', 0))}",
                str(record.get("quality_tier", "none")).replace("_", " "),
                "SELECTED" if selected else record.get("selection_reason", "—"),
            )
            event_details.append(
                "| " + " | ".join(_md_cell(v) for v in detail_values) + " |")
        event_details.extend(["", "</details>"])

    lines.extend([
        "",
        "## Event별 camera 상세",
        "",
        "각 Event를 펼치면 camera별 판정 근거를 확인할 수 있습니다.",
        *event_details,
        "",
    ])
    with open(path, "w", encoding="utf-8") as stream:
        stream.write("\n".join(lines))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session-root", default="data/session04/calib_train")
    parser.add_argument("--intrinsics-dir", default="intrinsics")
    parser.add_argument(
        "--output-dir",
        default="data/session04/calib_out/verify/cube_observation_quality",
    )
    parser.add_argument(
        "--policy", choices=("core_multiface", "legacy"),
        default="core_multiface",
    )
    parser.add_argument(
        "--include-gripped", action="store_true",
        help="Include captures where the robot is holding the cube.",
    )
    args = parser.parse_args()

    with open(os.path.join(args.session_root, "meta.json"), "r", encoding="utf-8") as stream:
        meta = json.load(stream)
    cfg, cfg_source = load_cube_config_from_meta(args.session_root)
    target = AprilTagCubeTarget(cfg)
    camera_ids = sorted(int(camera) for camera in meta["cam_indices"])
    K_map, D_map = {}, {}
    for camera in camera_ids:
        K_map[camera], D_map[camera], _ = load_intrinsics_with_depth_scale(
            args.intrinsics_dir, camera)

    observations, diagnostics = load_cube_pixel_observations(
        args.session_root, meta, target, K_map, D_map, camera_ids,
        int(meta["gripper_cam_idx"]),
        exclude_gripped=not args.include_gripped,
        observation_policy=args.policy,
    )
    report = {
        "session_root": os.path.abspath(args.session_root),
        "cube_config_source": cfg_source,
        "policy": args.policy,
        "exclude_gripped": not args.include_gripped,
        "selected_observation_count": len(observations),
        "diagnostics": diagnostics,
    }

    os.makedirs(args.output_dir, exist_ok=True)
    json_path = os.path.join(args.output_dir, "cube_observation_quality.json")
    camera_csv_path = os.path.join(args.output_dir, "event_camera_quality.csv")
    event_csv_path = os.path.join(args.output_dir, "event_summary.csv")
    html_path = os.path.join(args.output_dir, "cube_observation_quality_report.html")
    markdown_path = os.path.join(args.output_dir, "README.md")
    with open(json_path, "w", encoding="utf-8") as stream:
        json.dump(report, stream, indent=2, ensure_ascii=False)
    _write_csv(
        camera_csv_path,
        diagnostics["observation_quality_by_event_camera"],
        CAMERA_FIELDS,
    )
    _write_csv(event_csv_path, diagnostics["event_quality_summary"], EVENT_FIELDS)
    _write_html(html_path, report)
    _write_markdown(markdown_path, report, meta)

    print(json_path)
    print(camera_csv_path)
    print(event_csv_path)
    print(html_path)
    print(markdown_path)
    print(json.dumps({
        "selected_observation_count": len(observations),
        "comparison": diagnostics["available_observation_policy_comparison"],
        "selected_quality_tier_counts": diagnostics["selected_quality_tier_counts"],
    }, indent=2))


if __name__ == "__main__":
    main()
