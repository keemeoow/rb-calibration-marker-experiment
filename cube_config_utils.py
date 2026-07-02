# cube_config_utils.py
"""Small JSON helpers for CubeConfig.

Kept intentionally short: config.py is the single source of truth. This module
only serializes, deserializes, compares, and loads explicit config records.
Legacy image/meta inference was removed because it made the cube definition
hard to audit and can silently change geometry.
"""

import copy
import json
import os
from typing import List, Optional, Tuple

import numpy as np

from config import CubeConfig, get_default_cube_config


CONFIG_KEYS = (
    "cube_side_m",
    "marker_size_m",
    "marker_size_by_id",
    "marker_center_m",
    "marker_inset_m",
    "dictionary_name",
    "marker_ids",
    "id_to_face",
    "corner_reorder",
    "face_roll_deg",
    "marker_pose_4x4",
)


def clone_cube_config(cfg: CubeConfig) -> CubeConfig:
    return copy.deepcopy(cfg)


def cube_config_to_dict(cfg: CubeConfig) -> dict:
    return {
        "cube_side_m": float(cfg.cube_side_m),
        "marker_size_m": float(cfg.marker_size_m),
        "marker_size_by_id": {str(k): float(v) for k, v in cfg.marker_size_by_id.items()},
        "marker_center_m": {str(k): [float(x) for x in v] for k, v in cfg.marker_center_m.items()},
        "marker_inset_m": float(getattr(cfg, "marker_inset_m", 0.0)),
        "dictionary_name": str(cfg.dictionary_name),
        "marker_ids": [int(x) for x in cfg.marker_ids],
        "id_to_face": {str(k): str(v) for k, v in cfg.id_to_face.items()},
        "corner_reorder": {str(k): [int(x) for x in v] for k, v in cfg.corner_reorder.items()},
        "face_roll_deg": {str(k): float(v) for k, v in cfg.face_roll_deg.items()},
        "marker_pose_4x4": {
            str(k): [[float(x) for x in row] for row in np.asarray(v, dtype=np.float64).reshape(4, 4)]
            for k, v in getattr(cfg, "marker_pose_4x4", {}).items()
        },
    }


def _normalize(value):
    if isinstance(value, dict):
        return {str(k): _normalize(v) for k, v in sorted(value.items(), key=lambda kv: str(kv[0]))}
    if isinstance(value, (list, tuple)):
        return [_normalize(v) for v in value]
    if isinstance(value, float):
        return round(0.0 if abs(value) < 1e-12 else float(value), 9)
    return value


def cube_config_to_comparable_dict(cfg: CubeConfig) -> dict:
    return _normalize(cube_config_to_dict(cfg))


def cube_configs_equivalent(a: CubeConfig, b: CubeConfig) -> bool:
    return cube_config_to_comparable_dict(a) == cube_config_to_comparable_dict(b)


def cube_config_mismatch_keys(expected: CubeConfig, actual: CubeConfig) -> List[str]:
    exp = cube_config_to_comparable_dict(expected)
    act = cube_config_to_comparable_dict(actual)
    return [key for key in CONFIG_KEYS if exp.get(key) != act.get(key)]


def cube_config_from_dict(data: dict, base_cfg: Optional[CubeConfig] = None) -> CubeConfig:
    cfg = clone_cube_config(base_cfg or get_default_cube_config())
    if not isinstance(data, dict):
        return cfg

    cfg.cube_side_m = float(data.get("cube_side_m", cfg.cube_side_m))
    cfg.marker_size_m = float(data.get("marker_size_m", cfg.marker_size_m))
    cfg.marker_inset_m = float(data.get("marker_inset_m", getattr(cfg, "marker_inset_m", 0.0)))
    cfg.dictionary_name = str(data.get("dictionary_name", cfg.dictionary_name))
    cfg.marker_ids = tuple(int(x) for x in data.get("marker_ids", cfg.marker_ids))

    if "marker_size_by_id" in data:
        cfg.marker_size_by_id = {int(k): float(v) for k, v in data["marker_size_by_id"].items()}
    if "marker_center_m" in data:
        cfg.marker_center_m = {int(k): tuple(float(x) for x in v) for k, v in data["marker_center_m"].items()}
    if "id_to_face" in data:
        cfg.id_to_face = {int(k): str(v) for k, v in data["id_to_face"].items()}
    if "corner_reorder" in data:
        cfg.corner_reorder = {int(k): tuple(int(x) for x in v) for k, v in data["corner_reorder"].items()}
    if "face_roll_deg" in data:
        cfg.face_roll_deg = {int(k): float(v) for k, v in data["face_roll_deg"].items()}
    if "marker_pose_4x4" in data:
        cfg.marker_pose_4x4 = {
            int(k): np.asarray(v, dtype=np.float64).reshape(4, 4).tolist()
            for k, v in data["marker_pose_4x4"].items()
        }
    return cfg


def cube_config_from_search_result(data: dict, base_cfg: Optional[CubeConfig] = None) -> CubeConfig:
    """Accept old search-result shape but only read explicit config payloads."""
    cfg = clone_cube_config(base_cfg or get_default_cube_config())
    if not isinstance(data, dict):
        return cfg
    if isinstance(data.get("base_cube_config"), dict):
        cfg = cube_config_from_dict(data["base_cube_config"], cfg)
    elif isinstance(data.get("initial"), dict) and isinstance(data["initial"].get("base_cube_config"), dict):
        cfg = cube_config_from_dict(data["initial"]["base_cube_config"], cfg)
    return cube_config_from_dict(data.get("recommended", data), cfg)


def load_cube_config_from_json_file(path: str, default_cfg: Optional[CubeConfig] = None) -> Tuple[Optional[CubeConfig], str]:
    if not path or not os.path.exists(path):
        return None, "missing"
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
    except Exception:
        return None, "missing"

    base = clone_cube_config(default_cfg or get_default_cube_config())
    try:
        if isinstance(data, dict) and ("recommended" in data or "initial" in data):
            return cube_config_from_search_result(data, base), "json_search_result"
        return cube_config_from_dict(data, base), "json_config"
    except Exception:
        return None, "missing"


def load_cube_config_from_meta(root_folder: str, default_cfg: Optional[CubeConfig] = None) -> Tuple[CubeConfig, str]:
    cfg = clone_cube_config(default_cfg or get_default_cube_config())
    meta_path = os.path.join(root_folder, "meta.json")
    if not os.path.exists(meta_path):
        return cfg, "default"
    try:
        with open(meta_path, "r", encoding="utf-8") as f:
            meta = json.load(f)
    except Exception:
        return cfg, "default"
    if isinstance(meta.get("cube_config"), dict):
        try:
            return cube_config_from_dict(meta["cube_config"], cfg), "meta"
        except Exception:
            return cfg, "default"
    return cfg, "default"
