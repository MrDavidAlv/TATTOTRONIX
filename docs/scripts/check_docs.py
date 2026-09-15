"""Check that the published documents still agree with the data files.

A document and the run that produced it drift apart silently: someone reruns an
analysis, the numbers move, and the prose keeps quoting the old ones. Nothing
errors, and the document is now wrong in exactly the way that is hardest to
notice.

This checks the headline claims of docs/mathematical-model/ and the README
against docs/data/*.json, and fails if any of them no longer matches. It also
checks that every local link and image reference resolves, and that every
heading anchor a document links to exists.

    python3 docs/scripts/check_docs.py
"""

import json
import os
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
DATA = ROOT / "docs" / "data"
DOCS = [ROOT / "README.md"] + sorted((ROOT / "docs" / "mathematical-model").glob("*.md"))


def _spellings(x, nd=0, unit=""):
    """Every way the documents are allowed to write one number.

    Thousands appear both bare and space separated across the documents, and
    which one reads better depends on the sentence. The check is about the value
    being current, not about typography, so it accepts either.
    """
    plain = f"{x:,.{nd}f}"
    return {plain.replace(",", "") + unit, plain.replace(",", " ") + unit}


def claims():
    """(label, expected string, documents it must appear in) for each headline number."""
    s = json.loads((DATA / "summary.json").read_text())
    cs = json.loads((DATA / "control_study.json").read_text())
    cf = json.loads((DATA / "control_final.json").read_text())
    rec = cf["configs"]["wn40_pid+g+v"]
    pid_g = cs["modes"]["pid+g"]

    MM = "docs/mathematical-model/"
    out = [
        ("path points", _spellings(s["path_points"]), [MM + "toolpath.md", MM + "parameters.md"]),
        ("marked length", _spellings(s["path_marked_mm"], unit=" mm"),
         [MM + "toolpath.md", "README.md"]),
        ("travel length", _spellings(s["path_travel_mm"], unit=" mm"),
         [MM + "toolpath.md", "README.md"]),
        ("ik convergence", {"100%"}, [MM + "kinematics.md", MM + "parameters.md"]),
        ("panel reachable", {f'{s["panel_reachable_pct"]:.1f}%'},
         [MM + "kinematics.md", MM + "parameters.md"]),
        ("settled marking, logo start",
         {f'{s["cart_err_marking_settled_mean_um"]:.1f} µm'}, [MM + "control.md"]),
        ("worst marking, logo start",
         _spellings(s["cart_err_marking_max_um"], unit=" µm"), [MM + "control.md"]),
        ("pid+g settled, hard window",
         _spellings(pid_g["marking_settled_mean_um"], unit=" µm"), [MM + "control.md"]),
        ("recommended settled",
         _spellings(rec["marking_settled_mean_um"], unit=" µm"),
         [MM + "control.md", MM + "parameters.md"]),
        ("recommended worst",
         _spellings(rec["marking_max_um"], unit=" µm"),
         [MM + "control.md", MM + "parameters.md"]),
        ("recommended peak torque",
         {f'{rec["tau_peak"]:.2f} N·m'}, [MM + "control.md", MM + "parameters.md"]),
        ("needle entries", {"98"},
         [MM + "toolpath.md", MM + "control.md", MM + "parameters.md", "README.md"]),
    ]
    return out


def check_numbers():
    # Keyed by path, not by file name: there are two README.md.
    text = {str(p.relative_to(ROOT)): p.read_text(encoding="utf-8") for p in DOCS}
    bad = []
    for label, want, where in claims():
        for name in where:
            if not any(w in text.get(name, "") for w in want):
                bad.append(f"{name}: {label} should read one of "
                           + " or ".join(sorted(repr(w) for w in want)))
    return bad


def _anchors(path):
    out = set()
    for line in Path(path).read_text(encoding="utf-8").splitlines():
        m = re.match(r"^#{1,6}\s+(.*?)\s*$", line)
        if not m:
            continue
        t = re.sub(r"[`*$\\]", "", m.group(1)).lower()
        out.add(re.sub(r"\s+", "-", re.sub(r"[^\w\s-]", "", t).strip()))
    return out


def check_links():
    bad = []
    for dp, _, fs in os.walk(ROOT):
        if "/.git" in dp:
            continue
        for f in fs:
            if not f.endswith(".md"):
                continue
            fp = Path(dp) / f
            txt = fp.read_text(encoding="utf-8")
            rel = fp.relative_to(ROOT)
            refs = (re.findall(r"!\[[^\]]*\]\(([^)]+)\)", txt)
                    + re.findall(r'<img src="([^"]+)"', txt)
                    + re.findall(r"\[[^\]]*\]\(([^)#]+\.md)[^)]*\)", txt))
            for r in refs:
                if r.startswith("http"):
                    continue
                if not (Path(dp) / r).exists():
                    bad.append(f"{rel}: missing target {r}")
            for link in re.findall(r"\]\(([^)]*#[^)]+)\)", txt):
                if link.startswith("http"):
                    continue
                fpart, _, anch = link.partition("#")
                tgt = fp if not fpart else (Path(dp) / fpart)
                if tgt.exists() and anch not in _anchors(tgt):
                    bad.append(f"{rel}: broken anchor {link}")
    return bad


def main():
    nums, links = check_numbers(), check_links()
    for b in nums + links:
        print("  FAIL", b)
    print(f"  {len(claims())} documented numbers, "
          f"{'all match' if not nums else str(len(nums)) + ' stale'}")
    print(f"  links and anchors: {'all resolve' if not links else str(len(links)) + ' broken'}")
    return 1 if (nums or links) else 0


if __name__ == "__main__":
    sys.exit(main())
