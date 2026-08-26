#!/usr/bin/env python3
"""Single command surface for all real-data calibration comparisons.

Run ``python Run_calibration_comparison.py <command> --help`` for the options
owned by each experiment.  The default ``table1`` path and every supplementary
experiment share the modules under :mod:`calibration_pipeline`.
"""

from __future__ import annotations

import sys
from collections.abc import Callable


COMMANDS: dict[str, tuple[str, Callable[[list[str]], None]]] = {}


def _load_commands() -> dict[str, tuple[str, Callable[[list[str]], None]]]:
    from calibration_pipeline.blind_prediction import main as blind_prediction
    from calibration_pipeline.cross_target import main as cross_target
    from calibration_pipeline.external_gt import main as external_gt
    from calibration_pipeline.handeye_benchmark import main as handeye_benchmark
    from calibration_pipeline.marker_system import main as marker_system
    from calibration_pipeline.opencv_relative_baseline import (
        main as opencv_relative)
    from calibration_pipeline.table1 import main as table1

    return {
        "table1": (
            "A0~A5/B1~B3 Shared Baseline (동일 초기값) 비교", table1),
        "marker-system": ("board/cube/both modality별 end-to-end 비교", marker_system),
        "cross-target": (
            "Pre-GT Fixed-to-Fixed (고정카메라 간) 및 Gripper-to-Fixed "
            "(그리퍼카메라–고정카메라 간) Board/Cube 평가",
            cross_target),
        "opencv-relative": (
            "OpenCV PnP 기반 FK-free 고정카메라 reference baseline",
            opencv_relative),
        "handeye-benchmark": (
            "Session04 현재 방법과 OpenCV hand-eye/robot-world 7종 비교",
            handeye_benchmark),
        "blind-predict": ("외부 GT를 읽지 않는 blind 예측 생성", blind_prediction),
        "external-gt": ("잠금 해제된 외부 GT로 blind 예측 채점", external_gt),
    }


def _print_help(commands: dict[str, tuple[str, Callable[[list[str]], None]]]) -> None:
    print("Usage: python Run_calibration_comparison.py <command> [options]\n")
    print("Commands:")
    for name, (description, _) in commands.items():
        print(f"  {name:<14} {description}")
    print("\nUse '<command> --help' for command-specific options.")


def main(argv: list[str] | None = None) -> None:
    args = list(sys.argv[1:] if argv is None else argv)
    commands = _load_commands()
    if not args or args[0] in {"-h", "--help"}:
        _print_help(commands)
        return
    command = args.pop(0)
    if command not in commands:
        _print_help(commands)
        raise SystemExit(f"\nUnknown command: {command}")
    commands[command][1](args)


if __name__ == "__main__":
    main()
