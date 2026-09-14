"""촬영 데이터 폴더 이름을 한 곳에서 정한다.

규칙: 촬영 데이터는 **그 촬영을 하는 루트 폴더의 ``data/``** 안에
순차로 쌓이고, 폴더 이름에는 예외 없이 촬영 날짜 ``_MMDD`` 가 붙는다.
번호는 순차적으로만 올라가고 재사용하지 않는다.

    data/session11_zeus_wrist_motion_0914/                        # 메인 파이프라인
    ur3_calibration/data/session4_..._0914/                       # UR3 리그
    zeus_gello_calibration/data/session2_floor_board_dual_cam_0909/  # Zeus 리그

새 촬영 경로를 만드는 코드는 :data:`CAPTURE_DATA_ROOT` 와
:func:`session_folder_name` 에서 출발해야 하며, 날짜를 직접 적지 않는다.
"""

from __future__ import annotations

import re
from datetime import date
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
#: 모든 촬영 데이터가 모이는 단일 루트.
CAPTURE_DATA_ROOT = REPO_ROOT / "data"

SESSION_PREFIX = "session"
SESSION_DIGITS = 2
SESSION_DATE_FORMAT = "%m%d"
#: sessionNN / sessionNN_MMDD / sessionNN_<설명>_MMDD 를 모두 받아들인다.
SESSION_DIR_PATTERN = re.compile(
    rf"^{re.escape(SESSION_PREFIX)}(?P<index>[0-9]+)"
    r"(?:_(?P<label>.+?))?(?:_(?P<date>[0-9]{4}))?$")


def session_folder_name(index: int, label: str | None = None,
                        day: date | None = None) -> str:
    """Return ``sessionNN_<label>_<MMDD>`` — the only capture folder name form."""
    stamp = (day or date.today()).strftime(SESSION_DATE_FORMAT)
    slug = _slugify(label)
    middle = f"_{slug}" if slug else ""
    return f"{SESSION_PREFIX}{index:0{SESSION_DIGITS}d}{middle}_{stamp}"


def _slugify(label: str | None) -> str:
    if not label:
        return ""
    slug = re.sub(r"[^0-9A-Za-z]+", "_", label).strip("_").lower()
    return slug


def session_index(name: str) -> int | None:
    """Return the sequential index encoded in a capture folder name."""
    match = SESSION_DIR_PATTERN.fullmatch(name)
    return None if match is None else int(match.group("index"))


def existing_session_dirs(data_root: str | Path = CAPTURE_DATA_ROOT) -> list[Path]:
    """Return every capture session directory, in sequential order."""
    root = Path(data_root)
    if not root.is_dir():
        return []
    found = [(session_index(p.name), p) for p in root.iterdir() if p.is_dir()]
    return [p for index, p in sorted((i, p) for i, p in found if i is not None)]


def require_data_path_inside(path: str | Path, root: str | Path, *,
                             label: str = "capture path") -> Path:
    """Return an absolute path only when it is inside ``root``."""
    candidate = Path(path).expanduser().resolve()
    root = Path(root).resolve()
    if candidate != root and root not in candidate.parents:
        raise ValueError(f"{label} must be inside {root}; received {candidate}")
    return candidate


def require_capture_data_path(path: str | Path, *,
                              label: str = "capture path") -> Path:
    """Return an absolute path only when it is inside :data:`CAPTURE_DATA_ROOT`."""
    return require_data_path_inside(path, CAPTURE_DATA_ROOT, label=label)


def session_label(name: str) -> str:
    """Return the ``<설명>`` part of a capture folder name (``""`` when absent)."""
    match = SESSION_DIR_PATTERN.fullmatch(name)
    return "" if match is None else _slugify(match.group("label") or "")


def next_session_index(data_root: str | Path = CAPTURE_DATA_ROOT) -> int:
    """Return the next sequential session number — numbering never goes back."""
    used = [session_index(p.name) for p in existing_session_dirs(data_root)]
    return max([i for i in used if i is not None], default=0) + 1


def find_session_dir(label: str,
                     data_root: str | Path = CAPTURE_DATA_ROOT) -> Path | None:
    """Return the newest existing session folder captured with ``label``."""
    slug = _slugify(label)
    if not slug:
        return None
    matches = [p for p in existing_session_dirs(data_root)
               if session_label(p.name) == slug]
    return matches[-1] if matches else None


def resolve_session_dir(label: str,
                        data_root: str | Path = CAPTURE_DATA_ROOT,
                        *, create: bool = False) -> Path:
    """Return the folder for ``label`` — the existing one, else a new dated name.

    Resuming a capture must land back in the folder that already holds its
    frames, so an existing session with the same description wins.  Anything
    new gets ``session<NN>_<label>_<MMDD>`` with today's date.
    """
    found = find_session_dir(label, data_root)
    if found is not None:
        return found
    path = Path(data_root) / session_folder_name(next_session_index(data_root), label)
    if create:
        path.mkdir(parents=True, exist_ok=True)
    return path


def dated_name(base_name: str, day: date | None = None) -> str:
    """Return ``<base_name>_<MMDD>`` — the capture folder naming rule."""
    return f"{base_name}_{(day or date.today()).strftime(SESSION_DATE_FORMAT)}"


def find_dated_dir(base_name: str, parent: str | Path) -> Path | None:
    """Return the newest existing ``<base_name>_<MMDD>`` directory under ``parent``."""
    root = Path(parent)
    if not root.is_dir():
        return None
    matches = [p for p in root.iterdir()
               if p.is_dir() and re.fullmatch(re.escape(base_name) + r"(_[0-9]{4})?", p.name)]
    if not matches:
        return None
    return max(matches, key=lambda p: p.stat().st_mtime)


def resolve_dated_dir(base_name: str, parent: str | Path,
                      *, create: bool = False) -> Path:
    """Return ``parent/<base_name>_<MMDD>`` — the existing one, else today's.

    촬영을 이어서 하면 이미 프레임이 들어 있는 폴더로 돌아가야 하므로 같은
    이름의 기존 폴더가 있으면 그것을 쓰고, 처음 찍는 것만 오늘 날짜로 만든다.
    폴더가 어디에 있든(``data/``, ``zeus_gello_calibration/data/`` …)
    이름에 날짜가 붙는다는 규칙은 이 함수 하나로 지켜진다.
    """
    found = find_dated_dir(base_name, parent)
    if found is not None:
        return found
    path = Path(parent) / dated_name(base_name)
    if create:
        path.mkdir(parents=True, exist_ok=True)
    return path
