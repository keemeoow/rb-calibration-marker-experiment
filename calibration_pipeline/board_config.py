"""Fail-closed serialization helpers for the physical ChArUco board.

OpenCV's ``squares_x``/``squares_y`` count checker squares, while the number
of detectable ChArUco chessboard corners is ``(squares_x-1)*(squares_y-1)``.
Keeping that topology with the metric lengths in every capture artifact avoids
silently reinterpreting old images after a code-default change.
"""

from __future__ import annotations

import copy
import json
import os
from typing import List, Mapping, Optional, Tuple

from calibration_pipeline.config import (
    CharucoBoardConfig,
    get_default_charuco_board_config,
)


CONFIG_KEYS = (
    "squares_x",
    "squares_y",
    "square_length_m",
    "marker_length_m",
    "dictionary_name",
    "marker_id_start",
)


def clone_charuco_config(cfg: CharucoBoardConfig) -> CharucoBoardConfig:
    return copy.deepcopy(cfg)


def charuco_config_to_dict(cfg: CharucoBoardConfig) -> dict:
    return {
        "squares_x": int(cfg.squares_x),
        "squares_y": int(cfg.squares_y),
        "square_length_m": float(cfg.square_length_m),
        "marker_length_m": float(cfg.marker_length_m),
        "dictionary_name": str(cfg.dictionary_name),
        "marker_id_start": int(cfg.marker_id_start),
    }


def charuco_config_from_dict(
        data: Mapping, base_cfg: Optional[CharucoBoardConfig] = None,
) -> CharucoBoardConfig:
    if not isinstance(data, Mapping):
        raise ValueError("ChArUco config must be a mapping")
    missing = [key for key in CONFIG_KEYS if key not in data]
    if missing:
        raise ValueError(
            "ChArUco config is incomplete; missing " + ", ".join(missing))
    cfg = clone_charuco_config(base_cfg or get_default_charuco_board_config())
    cfg.squares_x = int(data["squares_x"])
    cfg.squares_y = int(data["squares_y"])
    cfg.square_length_m = float(data["square_length_m"])
    cfg.marker_length_m = float(data["marker_length_m"])
    cfg.dictionary_name = str(data["dictionary_name"])
    cfg.marker_id_start = int(data["marker_id_start"])
    validate_charuco_config(cfg)
    return cfg


def _normalized(cfg: CharucoBoardConfig) -> dict:
    result = charuco_config_to_dict(cfg)
    for key in ("square_length_m", "marker_length_m"):
        result[key] = round(float(result[key]), 12)
    return result


def charuco_configs_equivalent(
        first: CharucoBoardConfig, second: CharucoBoardConfig) -> bool:
    return _normalized(first) == _normalized(second)


def charuco_config_mismatch_keys(
        expected: CharucoBoardConfig, actual: CharucoBoardConfig) -> List[str]:
    first, second = _normalized(expected), _normalized(actual)
    return [key for key in CONFIG_KEYS if first[key] != second[key]]


def validate_charuco_config(cfg: CharucoBoardConfig) -> None:
    if int(cfg.squares_x) < 2 or int(cfg.squares_y) < 2:
        raise ValueError("ChArUco board needs at least 2x2 checker squares")
    if float(cfg.square_length_m) <= 0.0:
        raise ValueError("ChArUco square_length_m must be positive")
    if not 0.0 < float(cfg.marker_length_m) < float(cfg.square_length_m):
        raise ValueError(
            "ChArUco marker_length_m must be positive and smaller than "
            "square_length_m")
    if int(cfg.marker_id_start) < 0:
        raise ValueError("ChArUco marker_id_start must be non-negative")
    if not str(cfg.dictionary_name).startswith("DICT_"):
        raise ValueError("ChArUco dictionary_name must be an OpenCV DICT_* name")


def charuco_topology(cfg: CharucoBoardConfig) -> dict:
    return {
        "checker_squares_x": int(cfg.squares_x),
        "checker_squares_y": int(cfg.squares_y),
        "charuco_corner_columns": int(cfg.squares_x) - 1,
        "charuco_corner_rows": int(cfg.squares_y) - 1,
        "maximum_charuco_corners": (
            (int(cfg.squares_x) - 1) * (int(cfg.squares_y) - 1)),
        "checker_width_mm": (
            int(cfg.squares_x) * float(cfg.square_length_m) * 1000.0),
        "checker_height_mm": (
            int(cfg.squares_y) * float(cfg.square_length_m) * 1000.0),
    }


def load_charuco_config_from_meta(
        root_folder: str, *, require_frozen: bool = True,
        default_cfg: Optional[CharucoBoardConfig] = None,
) -> Tuple[CharucoBoardConfig, str]:
    cfg = clone_charuco_config(
        default_cfg or get_default_charuco_board_config())
    meta_path = os.path.join(root_folder, "meta.json")
    if not os.path.isfile(meta_path):
        if require_frozen:
            raise ValueError(f"capture metadata is missing: {meta_path}")
        return cfg, "default"
    with open(meta_path, "r", encoding="utf-8") as stream:
        meta = json.load(stream)
    data = meta.get("charuco_board_config")
    if not isinstance(data, Mapping):
        if require_frozen:
            raise ValueError(
                "meta.json has no frozen charuco_board_config; refusing to "
                "reinterpret captured images with current code defaults")
        return cfg, "default"
    return charuco_config_from_dict(data, cfg), "meta"
