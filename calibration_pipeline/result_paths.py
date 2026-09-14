"""ABLATION_TEST 결과 폴더 이름을 한 곳에서 정한다.

규칙: 결과는 항상 저장소 최상위의 ``ABLATION_TEST_result_<MMDD>`` 아래에
세션별로 들어간다 — ``ABLATION_TEST_result_0914/session07_zeus_.../``.
날짜를 폴더 이름에 박아 두면 같은 파이프라인을 여러 날 돌려도 어느 날 결과인지
이름만으로 구분된다.  경로를 새로 쓰는 코드는 반드시 :data:`ABLATION_RESULT_ROOT`
(또는 :func:`ablation_result_root`)에서 출발해야 하며, 날짜를 직접 적지 않는다.

의존성을 표준 라이브러리로만 유지한다 — tests/ 와 tools/ 어디서든 부담 없이
import 할 수 있어야 규칙이 실제로 한 곳에 모인다.
"""

from __future__ import annotations

import os
from datetime import date
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
ABLATION_RESULT_PREFIX = "ABLATION_TEST_result"
ABLATION_RESULT_DATE_FORMAT = "%m%d"
ABLATION_RESULT_ENV = "ABLATION_TEST_RESULT_ROOT"


def dated_result_name(day: date | None = None) -> str:
    """Return ``ABLATION_TEST_result_<MMDD>`` for ``day`` (default: today)."""
    stamp = (day or date.today()).strftime(ABLATION_RESULT_DATE_FORMAT)
    return f"{ABLATION_RESULT_PREFIX}_{stamp}"


def existing_result_roots() -> list[str]:
    """Return every ``ABLATION_TEST_result_<MMDD>`` directory, oldest write first."""
    found = [p for p in REPO_ROOT.glob(f"{ABLATION_RESULT_PREFIX}_[0-9][0-9][0-9][0-9]")
             if p.is_dir()]
    return [p.name for p in sorted(found, key=lambda p: p.stat().st_mtime)]


def ablation_result_root() -> str:
    """Return the result root directory name every stage must write under.

    A pipeline run spans several stages that read what the earlier ones wrote,
    so an existing dated root is reused (most recently written wins) rather
    than a fresh one appearing mid-run.  Only when none exists is today's name
    used.  Set ``ABLATION_TEST_RESULT_ROOT`` to pin a specific one.
    """
    override = os.environ.get(ABLATION_RESULT_ENV)
    if override:
        return override
    existing = existing_result_roots()
    return existing[-1] if existing else dated_result_name()


def new_ablation_result_root() -> str:
    """Return today's result root name, creating the directory if needed."""
    name = os.environ.get(ABLATION_RESULT_ENV) or dated_result_name()
    (REPO_ROOT / name).mkdir(parents=True, exist_ok=True)
    return name


#: 기본값 문자열에 그대로 끼워 쓰라고 미리 풀어 둔 값.
ABLATION_RESULT_ROOT = ablation_result_root()
