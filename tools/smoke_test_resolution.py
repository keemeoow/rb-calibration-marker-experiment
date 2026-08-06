#!/usr/bin/env python3
"""Can this host actually stream all four cameras at the target resolution?

Answering that on the capture PC *before* re-shooting intrinsics is the point:
Step1/Step1b take ~35 minutes, and every one of those minutes is wasted if the
four cameras cannot hold 1280x720 simultaneously afterwards.

The load here mirrors the real capture path in ``camera.py``: one thread per
camera doing ``wait_for_frames``, depth aligned to color, host-monotonic receipt
timestamps (independent RealSense clocks differ by minutes, so device clocks are
not comparable across cameras).

Standalone by design - no repo imports - so it can be copied to the capture PC
on its own. Needs only pyrealsense2 and numpy.

    # the decision: does 720p hold, and if not, does the depth downgrade rescue it?
    python tools/smoke_test_resolution.py

    # longer soak, only the two configs that matter
    python tools/smoke_test_resolution.py --duration 60 --configs 720p_depth720,720p_depth848

Exit code is 0 when the target config passes, 1 when it does not - so a wrapper
script can gate on it.
"""

from __future__ import annotations

import argparse
import collections
import json
import os
import statistics
import sys
import threading
import time
from typing import Dict, List, Optional

import numpy as np

try:
    import pyrealsense2 as rs
except ImportError:
    sys.exit("[ERROR] pyrealsense2 not installed. Run this on the capture PC.")


# --- thresholds from RECAPTURE_PROTOCOL / capture_gate ------------------------
SPAN_TARGET_MS = 50.0     # host timestamp span across cameras, goal
SPAN_LIMIT_MS = 120.0     # --max_capture_span_ms in Step2_capture.py
FPS_TOLERANCE = 0.90      # effective fps must reach this fraction of nominal


# Each config is (color w,h,fps) + optional (depth w,h,fps). Depth resolution is
# listed separately on purpose: camera.py currently forces depth to the color
# resolution, and whether that is what breaks the USB budget is exactly what
# this test is here to answer.
CONFIGS = {
    "720p_depth720": {
        "color": (1280, 720, 15), "depth": (1280, 720, 15),
        "note": "target; what camera.py does today (depth forced to color res)",
    },
    "720p_depth848": {
        "color": (1280, 720, 15), "depth": (848, 480, 15),
        "note": "mitigation; D435-native depth, ~40% less bandwidth, align still works",
    },
    "720p_nodepth": {
        "color": (1280, 720, 15), "depth": None,
        "note": "diagnostic; isolates whether depth is what breaks the budget",
    },
    "480p_depth480": {
        "color": (640, 480, 15), "depth": (640, 480, 15),
        "note": "control; the known-good config the current sessions were shot at",
    },
}
DEFAULT_ORDER = ["720p_depth720", "720p_depth848", "720p_nodepth", "480p_depth480"]


class CameraProbe:
    """One camera, one thread, counting what arrives - mirrors camera.py::_loop."""

    def __init__(self, serial: str, name: str, cfg: dict, align: bool):
        self.serial = serial
        self.name = name
        self.cfg_spec = cfg
        self.align_enabled = align

        self.lock = threading.Lock()
        self.running = False
        self.thread: Optional[threading.Thread] = None

        self.latest_host_ms: Optional[float] = None
        self.n_frames = 0
        self.n_dropped = 0          # gaps in the hardware frame counter
        self.n_errors = 0
        self.first_error: Optional[str] = None
        self._last_frame_number: Optional[int] = None
        self.arrival_gaps_ms: List[float] = []
        self._prev_arrival_ms: Optional[float] = None

        self.pipeline = rs.pipeline()
        self.rs_cfg = rs.config()
        self.rs_cfg.enable_device(serial)

        cw, ch, cfps = cfg["color"]
        self.rs_cfg.enable_stream(rs.stream.color, cw, ch, rs.format.bgr8, cfps)
        if cfg["depth"] is not None:
            dw, dh, dfps = cfg["depth"]
            self.rs_cfg.enable_stream(rs.stream.depth, dw, dh, rs.format.z16, dfps)

        self.align = (
            rs.align(rs.stream.color)
            if (cfg["depth"] is not None and align) else None
        )

    def start(self):
        self.pipeline.start(self.rs_cfg)
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _loop(self):
        while self.running:
            try:
                frames = self.pipeline.wait_for_frames(timeout_ms=2000)
                if self.align is not None:
                    frames = self.align.process(frames)

                color = frames.get_color_frame()
                if color is None:
                    continue
                depth = (frames.get_depth_frame()
                         if self.cfg_spec["depth"] is not None else None)

                # Copy the arrays. Skipping this would understate the real cost:
                # the capture path copies every frame it buffers, and at 720p
                # that memcpy is a meaningful share of the per-frame budget.
                _ = np.asanyarray(color.get_data()).copy()
                if depth is not None:
                    _ = np.asanyarray(depth.get_data()).copy()

                host_ms = time.monotonic() * 1000.0

                try:
                    fn = int(color.get_frame_number())
                    if self._last_frame_number is not None and fn > self._last_frame_number:
                        gap = fn - self._last_frame_number - 1
                        if gap > 0:
                            self.n_dropped += gap
                    self._last_frame_number = fn
                except Exception:
                    pass

                with self.lock:
                    self.latest_host_ms = host_ms
                    self.n_frames += 1
                    if self._prev_arrival_ms is not None:
                        self.arrival_gaps_ms.append(host_ms - self._prev_arrival_ms)
                    self._prev_arrival_ms = host_ms
            except Exception as exc:
                self.n_errors += 1
                if self.first_error is None:
                    self.first_error = f"{type(exc).__name__}: {exc}"
                time.sleep(0.005)

    def snapshot_host_ms(self) -> Optional[float]:
        with self.lock:
            return self.latest_host_ms

    def stop(self):
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=2.0)
        try:
            self.pipeline.stop()
        except Exception:
            pass


def enumerate_devices(expect: int) -> List[Dict]:
    ctx = rs.context()
    devs = []
    for d in ctx.query_devices():
        info = {"serial": d.get_info(rs.camera_info.serial_number),
                "name": d.get_info(rs.camera_info.name)}
        for key, attr in (("usb", "usb_type_descriptor"), ("fw", "firmware_version")):
            try:
                info[key] = d.get_info(getattr(rs.camera_info, attr))
            except Exception:
                info[key] = "?"
        devs.append(info)
    devs.sort(key=lambda x: x["serial"])

    print(f"[devices] found {len(devs)} (expected {expect})")
    for i, d in enumerate(devs):
        print(f"  cam{i}  {d['name']:24s} serial={d['serial']}  usb={d['usb']}  fw={d['fw']}")

    # A camera that negotiated USB 2.1 cannot do 720p color+depth at all, and it
    # fails as a confusing "unsupported profile" rather than as a cable problem.
    usb2 = [d for d in devs if str(d.get("usb", "")).startswith("2")]
    if usb2:
        print(f"\n  [!] {len(usb2)} camera(s) enumerated at USB 2.x: "
              f"{', '.join(d['serial'] for d in usb2)}")
        print("      720p color+depth is impossible on a USB 2.x link. Reseat the "
              "cable or move to a USB3 port before reading anything below.")
    if len(devs) != expect:
        print(f"\n  [!] expected {expect} cameras. The bandwidth verdict below only "
              f"applies to the {len(devs)} actually connected.")
    return devs


def supports(serial: str, stream, w: int, h: int, fps: int, fmt) -> bool:
    """Ask the device whether a profile exists, so unsupported != crash."""
    try:
        for dev in rs.context().query_devices():
            if dev.get_info(rs.camera_info.serial_number) != serial:
                continue
            for sensor in dev.query_sensors():
                for p in sensor.get_stream_profiles():
                    if p.stream_type() != stream:
                        continue
                    vp = p.as_video_stream_profile()
                    if (vp.width() == w and vp.height() == h
                            and p.fps() == fps and p.format() == fmt):
                        return True
    except Exception:
        return True  # can't tell; let pipeline.start be the judge
    return False


def run_config(name: str, spec: dict, devices: List[Dict],
               duration_s: float, align: bool, tick_ms: float) -> Dict:
    cw, ch, cfps = spec["color"]
    depth_txt = ("off" if spec["depth"] is None
                 else "{}x{}@{}".format(*spec["depth"]))
    print(f"\n{'='*72}\n[{name}] color {cw}x{ch}@{cfps}  depth {depth_txt}  "
          f"align={'on' if (align and spec['depth']) else 'off'}")
    print(f"  {spec['note']}")

    result = {"config": name, "color": spec["color"], "depth": spec["depth"],
              "status": "unknown", "cameras": {}}

    for d in devices:
        if not supports(d["serial"], rs.stream.color, cw, ch, cfps, rs.format.bgr8):
            print(f"  [SKIP] cam serial={d['serial']} has no color {cw}x{ch}@{cfps} profile")
            result["status"] = "unsupported"
            result["reason"] = f"color {cw}x{ch}@{cfps} unsupported on {d['serial']}"
            return result
        if spec["depth"] is not None:
            dw, dh, dfps = spec["depth"]
            if not supports(d["serial"], rs.stream.depth, dw, dh, dfps, rs.format.z16):
                print(f"  [SKIP] cam serial={d['serial']} has no depth {dw}x{dh}@{dfps} profile")
                result["status"] = "unsupported"
                result["reason"] = f"depth {dw}x{dh}@{dfps} unsupported on {d['serial']}"
                return result

    probes: List[CameraProbe] = []
    try:
        for i, d in enumerate(devices):
            p = CameraProbe(d["serial"], f"cam{i}", spec, align)
            p.start()
            probes.append(p)
            print(f"  started cam{i} ({d['serial']})")
    except Exception as exc:
        # The interesting failure: cameras 1-3 start, the 4th cannot get
        # bandwidth. That is the bandwidth budget answering the question.
        print(f"  [FAIL] only {len(probes)}/{len(devices)} cameras started: "
              f"{type(exc).__name__}: {exc}")
        for p in probes:
            p.stop()
        result["status"] = "start_failed"
        result["reason"] = f"{type(exc).__name__}: {exc}"
        result["n_started"] = len(probes)
        return result

    warmup_s = 3.0
    print(f"  warming up {warmup_s:.0f}s, then measuring {duration_s:.0f}s...")
    time.sleep(warmup_s)
    for p in probes:
        with p.lock:
            p.n_frames = 0
            p.n_dropped = 0
            p.arrival_gaps_ms.clear()

    spans_ms: List[float] = []
    t_end = time.monotonic() + duration_s
    t0 = time.monotonic()
    while time.monotonic() < t_end:
        time.sleep(tick_ms / 1000.0)
        # Exactly what Step2 does at a capture instant: take each camera's
        # latest frame and measure how far apart in host time they are.
        stamps = [p.snapshot_host_ms() for p in probes]
        if all(s is not None for s in stamps) and len(stamps) > 1:
            spans_ms.append(max(stamps) - min(stamps))
    elapsed = time.monotonic() - t0

    for i, p in enumerate(probes):
        with p.lock:
            n, dropped, errs = p.n_frames, p.n_dropped, p.n_errors
            gaps = list(p.arrival_gaps_ms)
        fps = n / elapsed if elapsed > 0 else 0.0
        result["cameras"][p.name] = {
            "serial": p.serial, "frames": n, "fps": round(fps, 2),
            "dropped": dropped, "errors": errs, "first_error": p.first_error,
            "max_gap_ms": round(max(gaps), 1) if gaps else None,
        }
        flag = "" if fps >= cfps * FPS_TOLERANCE else "   <-- BELOW NOMINAL"
        print(f"  cam{i}: {fps:5.2f} fps ({n} frames)  dropped={dropped}  "
              f"errors={errs}  max_gap={result['cameras'][p.name]['max_gap_ms']}ms{flag}")
        if p.first_error:
            print(f"        first error: {p.first_error}")

    for p in probes:
        p.stop()

    if spans_ms:
        spans_sorted = sorted(spans_ms)
        stats = {
            "p50": round(statistics.median(spans_sorted), 1),
            "p95": round(spans_sorted[int(0.95 * (len(spans_sorted) - 1))], 1),
            "max": round(max(spans_sorted), 1),
            "n_ticks": len(spans_sorted),
        }
        result["span_ms"] = stats
        print(f"  cross-camera host span: p50={stats['p50']}ms  p95={stats['p95']}ms  "
              f"max={stats['max']}ms  (target p50<={SPAN_TARGET_MS:.0f}, "
              f"hard limit {SPAN_LIMIT_MS:.0f})")

    fps_ok = all(c["fps"] >= cfps * FPS_TOLERANCE for c in result["cameras"].values())
    drops_ok = all(c["dropped"] == 0 for c in result["cameras"].values())
    errs_ok = all(c["errors"] == 0 for c in result["cameras"].values())
    span = result.get("span_ms")
    span_ok = (span is not None and span["max"] <= SPAN_LIMIT_MS)
    span_ideal = (span is not None and span["p50"] <= SPAN_TARGET_MS)

    result.update({"fps_ok": fps_ok, "drops_ok": drops_ok, "errors_ok": errs_ok,
                   "span_ok": span_ok, "span_ideal": span_ideal})

    if fps_ok and drops_ok and errs_ok and span_ok and span_ideal:
        result["status"] = "pass"
        print("  => PASS")
    elif fps_ok and span_ok:
        result["status"] = "marginal"
        reasons = []
        if not drops_ok:
            reasons.append("dropped frames")
        if not errs_ok:
            reasons.append("stream errors")
        if not span_ideal:
            reasons.append(f"span p50 > {SPAN_TARGET_MS:.0f}ms")
        result["reason"] = ", ".join(reasons)
        print(f"  => MARGINAL ({result['reason']})")
    else:
        result["status"] = "fail"
        reasons = []
        if not fps_ok:
            reasons.append("fps below nominal")
        if not span_ok:
            reasons.append(f"span max > {SPAN_LIMIT_MS:.0f}ms")
        if not drops_ok:
            reasons.append("dropped frames")
        if not errs_ok:
            reasons.append("stream errors")
        result["reason"] = ", ".join(reasons)
        print(f"  => FAIL ({result['reason']})")
    return result


def verdict(results: List[Dict], target: str) -> int:
    print(f"\n{'='*72}\nSUMMARY\n{'='*72}")
    print(f"{'config':18s} {'status':12s} {'min fps':>8s} {'drops':>6s} {'span p50/max':>14s}")
    for r in results:
        if r["status"] in ("unsupported", "start_failed"):
            print(f"{r['config']:18s} {r['status']:12s} {'-':>8s} {'-':>6s} {'-':>14s}"
                  f"   {r.get('reason','')}")
            continue
        cams = r["cameras"].values()
        min_fps = min(c["fps"] for c in cams) if cams else 0
        drops = sum(c["dropped"] for c in cams)
        span = r.get("span_ms")
        span_txt = f"{span['p50']}/{span['max']}" if span else "-"
        print(f"{r['config']:18s} {r['status']:12s} {min_fps:8.2f} {drops:6d} {span_txt:>14s}")

    by_name = {r["config"]: r for r in results}
    tgt = by_name.get(target)
    print()

    if tgt is None:
        print(f"[verdict] target config '{target}' was not run.")
        return 1

    if tgt["status"] == "pass":
        print(f"[verdict] {target} PASSES. Go ahead with the high-resolution "
              f"intrinsic re-shoot; no code change needed.")
        return 0

    fallback = by_name.get("720p_depth848")
    nodepth = by_name.get("720p_nodepth")
    control = by_name.get("480p_depth480")

    print(f"[verdict] {target} did NOT pass ({tgt.get('reason', tgt['status'])}).")
    if fallback is not None and fallback["status"] in ("pass", "marginal"):
        print("          720p color + 848x480 depth held up, so this is the USB "
              "bandwidth budget, not the host.")
        print("          Fix: let depth resolution differ from color. camera.py:74-75 "
              "and Step1_dump_all_intrinsics.py:141-142 force them equal;")
        print("          rs.align handles differing resolutions, so this is a small change.")
    elif nodepth is not None and nodepth["status"] == "pass":
        print("          720p held up only with depth off entirely. Depth at 720p does "
              "not fit; either split the cameras across USB controllers")
        print("          or drop depth resolution further.")
    elif control is not None and control["status"] == "pass":
        print("          Only 640x480 held up. 720p on four cameras needs a hardware "
              "change (separate USB3 controllers / powered hubs)")
        print("          before the high-resolution plan is viable.")
    else:
        print("          Even the 640x480 control did not pass, so something is wrong "
              "beyond resolution - check USB link speed and cables first.")
    return 1


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--configs", default=",".join(DEFAULT_ORDER),
                    help=f"comma-separated, in order. available: {', '.join(CONFIGS)}")
    ap.add_argument("--target", default="720p_depth720",
                    help="config whose result decides the exit code")
    ap.add_argument("--duration", type=float, default=20.0,
                    help="measured seconds per config, after a 3s warmup (default 20)")
    ap.add_argument("--expect-cameras", type=int, default=4)
    ap.add_argument("--tick-ms", type=float, default=200.0,
                    help="how often to sample the cross-camera span (default 200)")
    ap.add_argument("--no-align", action="store_true",
                    help="skip depth->color alignment (isolates its CPU cost)")
    ap.add_argument("--reset", action="store_true",
                    help="hardware-reset all devices first (after a prior crash)")
    ap.add_argument("--json-out", default=None, help="write full results here")
    args = ap.parse_args()

    names = [n.strip() for n in args.configs.split(",") if n.strip()]
    unknown = [n for n in names if n not in CONFIGS]
    if unknown:
        sys.exit(f"[ERROR] unknown config(s): {', '.join(unknown)}\n"
                 f"        available: {', '.join(CONFIGS)}")

    print(f"host: {os.uname().sysname} {os.uname().release}  cpus={os.cpu_count()}")
    print(f"plan: {' -> '.join(names)}   {args.duration:.0f}s each\n")

    if args.reset:
        print("[reset] hardware-resetting all devices...")
        for d in rs.context().query_devices():
            try:
                d.hardware_reset()
            except Exception:
                pass
        time.sleep(8.0)

    devices = enumerate_devices(args.expect_cameras)
    if not devices:
        sys.exit("[ERROR] no RealSense devices found.")

    results = []
    for name in names:
        try:
            results.append(run_config(name, CONFIGS[name], devices,
                                      args.duration, not args.no_align, args.tick_ms))
        except KeyboardInterrupt:
            print("\n[abort] interrupted")
            break
        # Let USB settle so one config's teardown does not taint the next.
        time.sleep(2.0)

    code = verdict(results, args.target)

    if args.json_out:
        os.makedirs(os.path.dirname(os.path.abspath(args.json_out)) or ".", exist_ok=True)
        with open(args.json_out, "w") as f:
            json.dump({"devices": devices, "results": results,
                       "duration_s": args.duration, "align": not args.no_align}, f, indent=2)
        print(f"\n[ok] wrote {args.json_out}")
    return code


if __name__ == "__main__":
    sys.exit(main())
