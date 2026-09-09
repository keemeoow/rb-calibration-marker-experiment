from __future__ import annotations

import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
FINAL_REPORT_DIR = ROOT / "ABLATION_TEST_result/session04/ABLATION_TEST_table1"

# Serialized enum values such as ``raw-FK-fixed`` remain compatibility-only
# identifiers. They must never leak into reader-facing reports or documents.
FORBIDDEN_PUBLIC_TERMS = re.compile(
    r"\b(?:no[- ]?fk|raw[- ]?fk|vision[- ]?only|visual[- ]?only|"
    r"fk[- ]?free|fixed[- ]?fk)\b|"
    r"vision[- ]aligned[ -]?fk|aligned[- ]fk|fk 후보정|corrected fk|"
    r"(?<!corrected-)\bfk soft factor\b|"
    r"(?<!corrected-)\bfk factor\b|−FK",
    re.IGNORECASE,
)


def _public_documents() -> list[Path]:
    documents = sorted(ROOT.rglob("*.md"))
    documents.extend([
        FINAL_REPORT_DIR / "ABLATION_TEST_table1_results.csv",
        FINAL_REPORT_DIR / "calibration_summary.csv",
        FINAL_REPORT_DIR / "ABLATION_TEST_TABLE1_INTERACTIVE.html",
    ])
    return documents


def test_public_documents_use_canonical_fk_terminology():
    violations = []
    for path in _public_documents():
        text = path.read_text(encoding="utf-8-sig")
        for line_number, line in enumerate(text.splitlines(), start=1):
            match = FORBIDDEN_PUBLIC_TERMS.search(line)
            if match:
                violations.append(
                    f"{path.relative_to(ROOT)}:{line_number}: {match.group(0)!r}")

    assert not violations, "Non-canonical public terminology:\n" + "\n".join(
        violations)


def test_final_report_exposes_the_three_canonical_method_families():
    report = (FINAL_REPORT_DIR / "ABLATION_TEST_TABLE1_RESULTS.md").read_text(
        encoding="utf-8")
    assert "VISION (cube pose free)" in report
    assert "A3 (FK hard fixed)" in report
    assert "A4 (corrected-FK soft factor)" in report
    assert "A5 (corrected-FK hard fixed (VISION-aligned))" in report
