# Copyright 2026 Mario David Alvarez Vallejo
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Check that the published documents still agree with the data files.

A document and the run that produced it drift apart silently: someone reruns an
analysis, the numbers move, and the prose keeps quoting the old ones. Nothing
errors, and the document is now wrong in exactly the way that is hardest to
notice.

This checks the headline claims of docs/mathematical-model/ and the README
against docs/data/*.json, and fails if any of them no longer matches. Beyond
those, every figure in micrometres or newton-metres anywhere in the published
documents has to be found in the data at the precision it is written to, or be
listed in DECLARED with the reason it is allowed not to be. It also
checks that every local link and image reference resolves, that every heading
anchor a document links to exists, and that the paths the scripts compute for
themselves still land inside the repository.

    python3 docs/scripts/check_docs.py
"""

import json
import os
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
DATA = ROOT / "docs" / "data"
DOCS = ([ROOT / "README.md", ROOT / "docs" / "moveit.md"]
        + sorted((ROOT / "docs" / "mathematical-model").glob("*.md")))


def _spellings(x, unit="", decimals=(0, 1)):
    """Every way the documents are allowed to write one number.

    Thousands appear both bare and space separated, and a value is written to
    zero or one decimal depending on whether the sentence is about its precision.
    The check is about the value being current, not about typography, so it
    accepts any of those spellings and none of the wrong ones.
    """
    out = set()
    for nd in decimals:
        plain = f"{x:,.{nd}f}"
        out.add(plain.replace(",", "") + unit)
        out.add(plain.replace(",", " ") + unit)
    return out


def claims():
    """(label, expected string, documents it must appear in) for each headline number."""
    s = json.loads((DATA / "summary.json").read_text())
    cs = json.loads((DATA / "control_study.json").read_text())
    cf = json.loads((DATA / "control_final.json").read_text())
    # The recommended configuration is the one with the smallest worst case,
    # picked from the file rather than named here: hardcoding the key meant the
    # check went stale the moment the recommendation moved, which is the one
    # thing it exists to prevent.
    rec = min(cf["configs"].values(), key=lambda c: c["marking_max_um"])
    pid_g = cs["modes"]["pid+g"]
    lo, hi = s["sampled_boundary_band_wnT"]

    mv = json.loads((DATA / "moveit_check.json").read_text(encoding="utf-8"))
    MM = "docs/mathematical-model/"
    out = [
        ("path points", _spellings(s["path_points"], decimals=(0,)),
         [MM + "toolpath.md", MM + "parameters.md"]),
        ("marked length", _spellings(s["path_marked_mm"], unit=" mm"),
         [MM + "toolpath.md", "README.md"]),
        ("travel length", _spellings(s["path_travel_mm"], unit=" mm"),
         [MM + "toolpath.md", "README.md"]),
        ("ik convergence", {"100%"}, [MM + "kinematics.md", MM + "parameters.md"]),
        ("panel reachable", {f'{s["panel_reachable_pct"]:.1f}%'},
         [MM + "kinematics.md", MM + "parameters.md"]),
        ("settled marking, logo start",
         _spellings(s["cart_err_marking_settled_mean_um"], unit=" µm"),
         [MM + "control.md"]),
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
        # Read from the data. This used to be the literal "98", so the check
        # compared the documents with themselves and passed for as long as
        # they agreed - long after the path had come to have 117 entries.
        ("needle entries", {str(s["needle_entries"])},
         [MM + "toolpath.md", MM + "control.md", MM + "parameters.md", "README.md"]),
        ("tracer outer perimeter",
         {f'{s["tracer_annulus_perimeters_mm"][0]:.2f} mm'}, [MM + "toolpath.md"]),
        ("tracer inner perimeter",
         {f'{s["tracer_annulus_perimeters_mm"][1]:.2f} mm'}, [MM + "toolpath.md"]),
        ("logo regions", {f'{s["tracer_logo_regions"]} regions'}, [MM + "toolpath.md"]),
        ("single-axis step overshoot", {f'{s["step_overshoot_single_axis_pct"]:.0f}%'},
         [MM + "control.md"]),
        ("joint_1 step overshoot", {f'by {s["step_overshoot_ff_pct"][0]:.0f}%'},
         [MM + "control.md"]),
        ("joint_5 step overshoot", {f'by {s["step_overshoot_ff_pct"][4]:.0f}%'},
         [MM + "control.md"]),
        ("coupled modes, gain range",
         {f'from {s["modal_gain"][-1]:.2f} to {s["modal_gain"][0]:.2f}'}, [MM + "control.md"]),
        ("stiffest mode's gain", {f'{s["modal_gain"][0]:.2f} times its designed gain'},
         [MM + "control.md"]),
        ("softest mode's damping", {f'damping ratio of {s["coupled_zeta_min"]:.2f}'},
         [MM + "control.md"]),
        ("sampled boundary, tuning pose",
         {f'{s["sampled_boundary_wnT"]:.3f} at the tuning pose'},
         [MM + "control.md", MM + "parameters.md"]),
        ("sampled boundary, along the drawing",
         {f'between {lo:.3f} and {hi:.3f}', f'{lo:.3f} to {hi:.3f}'},
         [MM + "control.md", MM + "parameters.md"]),
        ("drawing animation length", {f'{s["path_time_s"]:.0f} s compressed into 24'},
         [MM + "toolpath.md", "README.md"]),
        ("moveit plan waypoints", {f'{mv["plan"]["waypoints"]} waypoints over '
                                   f'{mv["plan"]["duration_s"]:.2f} s'}, ["docs/moveit.md"]),
        ("moveit planning time", {f'{mv["plan"]["planning_time_s"] * 1000:.0f} ms'},
         ["docs/moveit.md", "README.md"]),
    ] + [
        (f"gravity check, pose {i}",
         {f"{v:.1e}".split("e")[0] + " × 10" + str(int(f"{v:.1e}".split("e")[1])).translate(
             str.maketrans("-0123456789", "⁻⁰¹²³⁴⁵⁶⁷⁸⁹")) + " N·m"}, [MM + "control.md"])
        for i, v in enumerate(s["gravity_check_Nm"])
    ]
    return out


def check_numbers():
    # Keyed by path, not by file name: there are two README.md.
    text = {str(p.relative_to(ROOT)): p.read_text(encoding="utf-8") for p in DOCS}
    bad = []
    for label, want, where in claims():
        for name in where:
            # Anchored on both sides: as a bare substring "98" was found inside
            # "98.4%", and the entry count passed in a document that said 117.
            # Whitespace is normalised, because markdown wraps its lines wherever
            # they happen to run out, and a claim is still the claim across a break.
            body = re.sub(r"\s+", " ", text.get(name, ""))
            if not any(re.search(r"(?<![\d.])" + re.escape(w) + r"(?![\d])", body)
                       for w in want):
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


# ---------------------------------------------------------------------------
# Traceability: every measured figure has to come from somewhere.
#
# The claims above certify a dozen headline numbers by name. The documents
# carry well over a hundred more, in tables and in prose, and naming each one
# would never keep up. So this checks them by membership instead: every figure
# written in um or N.m must equal some value in docs/data/*.json at the
# precision it is written to. It is a necessary condition, not a sufficient
# one - a stale number can coincide with an unrelated value - but a figure that
# matches nothing in the data cannot have come from it, and that is the failure
# that matters: when the model changes, almost every old number stops existing.
#
# It found, on its first run, a subsection that contradicted the table above it
# on three counts and a caption that had rounded 94.67 to 94.
# ---------------------------------------------------------------------------

TRACED = [ROOT / "README.md", ROOT / "docs" / "moveit.md"] + sorted(
    (ROOT / "docs" / "mathematical-model").glob("*.md"))

# Digit groups may be separated by a space, thin space or narrow no-break
# space, as in "30 037 um"; read as one number, not as "037". A leading minus,
# ASCII or typographic, is part of the figure when it is not glued to a word:
# "-0.5424 N.m" is a signed value, "10-20 um" is a range.
_FIGURE = re.compile(
    r"(?<![\w.])([\u2212-]?)"
    r"(\d{1,3}(?:[ \u2009\u202f]\d{3})+|\d+(?:\.\d+)?)\s?(µm|N·m)")

#: Figures the documents may state without a data file behind them, keyed by
#: file and by the figure exactly as written, each with its reason. The key is
#: that narrow on purpose: an entry excuses one figure in one document, not the
#: same number wherever it turns up. An entry that no longer matches anything
#: is itself a failure, so this list cannot only ever grow.
DECLARED = {
    ("docs/mathematical-model/control.md", "300 µm"):
        "the tattoo line width, a specification; and the worked example "
        "e = v / wn at the marking feed and the analysis bandwidth, which "
        "happens to equal it - that coincidence is the point being made",
    ("docs/mathematical-model/control.md", "164.5 µm"):
        "history: the torque split validated on the stand-in artwork, which "
        "the repository no longer carries",
    ("docs/mathematical-model/control.md", "165 µm"):
        "history: the same validation, from control_study.py on the stand-in",
    ("docs/mathematical-model/control.md", "2717 µm"):
        "history: the first attempt at letting the loop settle after an "
        "entry, kept because it is the attempt that did not work",
    ("docs/mathematical-model/control.md", "22.5 µm"):
        "history: first row of 'How this number moved', the stand-in artwork",
    ("docs/mathematical-model/control.md", "272 µm"):
        "history: the worst case on that same stand-in row, superseded when "
        "the drawing moved to the real logo",
}


#: Data files that carry no figure in um or N.m, and so are not allowed to
#: vouch for one. The membership test compares numbers, not units, and a mass
#: file full of areas and volumes will contain something that rounds to almost
#: any figure: upper_arm_link's 299.57 cm^2 of surface once "traced" the 300 um
#: line width.
NOT_MEASURED_IN_UM_OR_NM = {"mass_properties.json"}


def _data_values():
    """Every number in every data file that carries um or N.m figures."""
    found = []

    def walk(obj):
        if isinstance(obj, bool):
            return
        if isinstance(obj, (int, float)):
            found.append(float(obj))
        elif isinstance(obj, dict):
            for v in obj.values():
                walk(v)
        elif isinstance(obj, list):
            for v in obj:
                walk(v)

    for f in sorted(DATA.glob("*.json")):
        if f.name in NOT_MEASURED_IN_UM_OR_NM:
            continue
        walk(json.loads(f.read_text(encoding="utf-8")))
    return found


def _traceable(written, data, sign=""):
    """Whether a figure, as written, rounds from some value in the data.

    With a sign written, the signed value has to match. Without one, the
    figure is read as a magnitude, which is how documents quote a peak torque
    or an error whatever direction it had in the data.
    """
    digits = re.sub(r"[ \u2009\u202f]", "", written)
    decimals = len(digits.split(".")[1]) if "." in digits else 0
    value = float(digits)
    half = 0.5 * 10 ** -decimals + 1e-9
    if sign:
        return any(abs(v + value) <= half for v in data)
    return any(abs(abs(v) - value) <= half for v in data)


def check_traceability():
    """Every um and N.m figure in the documents, looked up in the data."""
    data = _data_values()
    bad, checked, used = [], 0, set()
    for doc in TRACED:
        rel = str(doc.relative_to(ROOT))
        for n, line in enumerate(doc.read_text(encoding="utf-8").splitlines(), 1):
            for m in _FIGURE.finditer(line):
                checked += 1
                sign, number, unit = m.group(1), m.group(2), m.group(3)
                figure = f"{sign}{number} {unit}"
                ok = _traceable(number, data, sign)
                if (rel, figure) in DECLARED:
                    used.add((rel, figure))
                    if ok:
                        bad.append(f"{rel}:{n}: {figure} is exempt but is now in "
                                   "the data; remove its DECLARED entry")
                    continue
                if not ok:
                    bad.append(f"{rel}:{n}: {figure} is in no data file")
    for key in sorted(set(DECLARED) - used):
        bad.append(f"DECLARED entry matches nothing any more: {key[0]}: {key[1]}")
    return bad, checked, len(used)


def check_gains():
    """The gains parameters.md publishes are the ones summary.json holds.

    They carry no unit, so the traceability check never sees them, and they
    had drifted: the table gave Kp = 32.24 against 32.66 in the data, from a
    tuning two revisions old. Each written gain has to round, at the precision
    it is written to, from the value in the same position in the data.
    """
    summary = json.loads((DATA / "summary.json").read_text(encoding="utf-8"))
    doc = ROOT / "docs" / "mathematical-model" / "parameters.md"
    text = doc.read_text(encoding="utf-8")
    bad = []
    for key, symbol in (("Kp", "K_p"), ("Ki", "K_i"), ("Kd", "K_d")):
        row = re.search(r"\|\s*\$" + symbol + r"\$\s*\|\s*`([^`]+)`", text)
        if row is None:
            bad.append(f"parameters.md: no {symbol} row")
            continue
        written = [w.strip() for w in row.group(1).split(",")]
        data = summary[key]
        if len(written) != len(data):
            bad.append(f"parameters.md: {symbol} lists {len(written)} gains, "
                       f"the data has {len(data)}")
            continue
        for i, (w, v) in enumerate(zip(written, data), 1):
            if not _traceable(w, [v]):
                bad.append(f"parameters.md: {symbol} joint {i} is written {w}, "
                           f"the data says {v}")
    return bad


def check_shares():
    """Percentages of the torque limit, which neither check above can see.

    A share of the limit is a derived figure with no unit, so the
    traceability check never reads it, and it moves every time the masses
    do. Two are published. The planner's worst case comes from the header of
    the generated joint_limits.yaml, which test_moveit_config already holds
    to its generator. The drawing's is the recommended configuration's peak
    torque over the effort limit, read from the description rather than
    written here.
    """
    bad = []
    limits = (ROOT / "src" / "tattotronix_moveit_config" / "config"
              / "joint_limits.yaml").read_text(encoding="utf-8")
    planner = re.search(r"Worst case across the arm is ([\d.]+)%", limits)
    arm = (ROOT / "src" / "tattotronix_description" / "urdf"
           / "arm_macro.xacro").read_text(encoding="utf-8")
    effort = float(re.search(r'name="joint_effort" value="([\d.]+)"', arm).group(1))
    final = json.loads((DATA / "control_final.json").read_text(encoding="utf-8"))
    rec = min(final["configs"].values(), key=lambda c: c["marking_max_um"])
    drawing = 100.0 * rec["tau_peak"] / effort

    expect = [
        (planner.group(1) + "% of the available", ["docs/moveit.md", "README.md"]),
        (f"{drawing:.0f}% of the {effort:.0f} N·m",
         ["docs/mathematical-model/control.md",
          "docs/mathematical-model/parameters.md"]),
    ]
    for phrase, docs in expect:
        for doc in docs:
            text = re.sub(r"\s+", " ", (ROOT / doc).read_text(encoding="utf-8"))
            # Anchored so that 1.6% is not found inside 11.6%.
            if not re.search(r"(?<![\d.])" + re.escape(phrase), text):
                bad.append(f"{doc}: expected '{phrase}'")
    return bad


def check_inertia_table():
    """control.md's effective-against-diagonal table, and the factors it quotes.

    The table is in kg m^2, which the traceability check does not read, and it
    moves with every change to the masses. Each cell has to round from the
    zero-pose value in summary.json, and the two factors the prose draws from
    it - how far the diagonal overstates joints 2 and 3 - have to be the ratio
    of the two columns.
    """
    s = json.loads((DATA / "summary.json").read_text(encoding="utf-8"))
    if "J_eff_zero" not in s:
        return ["summary.json has no zero-pose inertias; re-run analysis.py"]
    text = (ROOT / "docs" / "mathematical-model" / "control.md").read_text(encoding="utf-8")
    bad = []
    for i in range(5):
        row = re.search(r"\|\s*`joint_%d`\s*\|\s*([\d.]+)\s*\|\s*([\d.]+)\s*\|" % (i + 1), text)
        if row is None:
            bad.append(f"control.md: no inertia row for joint_{i + 1}")
            continue
        for written, value, what in ((row.group(1), s["J_eff_zero"][i], "effective"),
                                     (row.group(2), s["M_diag_zero"][i], "diagonal")):
            if not _traceable(written, [value]):
                bad.append(f"control.md: joint_{i + 1} {what} inertia is written "
                           f"{written}, the data says {value:.6g}")
    factors = re.search(r"differ by a factor of ([\d.]+) on `joint_2` and ([\d.]+) on "
                        r"`joint_3`", re.sub(r"\s+", " ", text))
    if factors is None:
        bad.append("control.md: the joint_2 and joint_3 factors are not stated")
    else:
        for written, j in ((factors.group(1), 1), (factors.group(2), 2)):
            ratio = s["M_diag_zero"][j] / s["J_eff_zero"][j]
            if not _traceable(written, [ratio]):
                bad.append(f"control.md: joint_{j + 1} factor is written {written}, "
                           f"the data gives {ratio:.3f}")
    j5 = re.search(r"and by ([\d.]+) on `joint_5`", re.sub(r"\s+", " ", text))
    ratio5 = s["M_diag_zero"][4] / s["J_eff_zero"][4]
    if j5 is None or not _traceable(j5.group(1), [ratio5]):
        bad.append(f"control.md: joint_5 factor should read {ratio5:.1f}")
    return bad


def check_mass():
    """mass.md against mass_properties.json: its table cell by cell, and the
    headline figures it states in prose. None of them is in um or N.m, so the
    traceability check never reads them.
    """
    f = DATA / "mass_properties.json"
    doc = ROOT / "docs" / "mathematical-model" / "mass.md"
    if not f.exists() or not doc.exists():
        return [f"{f.name} or {doc.name} is missing"]
    m = json.loads(f.read_text(encoding="utf-8"))
    text = doc.read_text(encoding="utf-8")
    flat = re.sub(r"\s+", " ", text)
    bad = []
    for name, v in m["links"].items():
        row = re.search(r"\|\s*`%s`\s*\|([^\n]+)" % re.escape(name), text)
        if row is None:
            bad.append(f"mass.md: no row for {name}")
            continue
        cells = [re.sub(r"[^\d.]", "", c) for c in row.group(1).split("|")[:6]]
        want = [v["volume_cm3"], v["box_mass_kg"] * 1000, v["box_implied_density_g_cm3"],
                v["shell_kg"] * 1000, v["servos_kg"] * 1000, v["total_kg"] * 1000]
        for label, c, w in zip(("volume", "box mass", "density", "shell", "servos", "total"),
                               cells, want):
            if not c or not _traceable(c, [w]):
                bad.append(f"mass.md: {name} {label} is written {c}, the data says {w:.4g}")
    for phrase in (f"{m['total_kg']:.3f} kg", f"{m['box_total_kg']:.3f} kg",
                   f"{m['servo_share_pct']:.1f}%", f"{m['box_implied_density_g_cm3']:.2f} g/cm³",
                   f"factor of {m['box_over_printed']:.1f}"):
        if not re.search(r"(?<![\d.])" + re.escape(phrase), flat):
            bad.append(f"mass.md: expected '{phrase}'")
    return bad


def check_kinematics():
    """kinematics.md and parameters.md against kinematics_study.json.

    Counts, singular values and condition numbers carry no unit, so nothing
    else reads them; the page went on quoting a 4372-point path long after the
    path had 16 656.
    """
    f = DATA / "kinematics_study.json"
    if not f.exists():
        return ["kinematics_study.json is missing; run kinematics_study.py"]
    k = json.loads(f.read_text(encoding="utf-8"))
    p = k["path"]
    mm = ROOT / "docs" / "mathematical-model"
    kin = re.sub(r"\s+", " ", (mm / "kinematics.md").read_text(encoding="utf-8"))
    par = re.sub(r"\s+", " ", (mm / "parameters.md").read_text(encoding="utf-8"))
    want = [
        (kin, "kinematics.md", f"all {p['points']} path points"),
        (kin, "kinematics.md", f"{p['sigma1_min']:.4f} … {p['sigma1_max']:.4f}"),
        (kin, "kinematics.md", f"{p['sigma5_min']:.4f} … {p['sigma5_max']:.4f}"),
        (kin, "kinematics.md", f"{p['cond_min']:.0f} … {p['cond_max']:.0f}"),
        (par, "parameters.md", f"{p['sigma5_min']:.4f} … {p['sigma5_max']:.4f}"),
        (par, "parameters.md", f"{p['cond_min']:.0f} … {p['cond_max']:.0f}"),
    ]
    for r in k["placements"]:
        want.append((kin, "kinematics.md",
                     f"| {r['width_mm']} × {r['offset_mm']} mm | {r['reach_mm']:.0f} mm | "
                     f"{r['sigma5_min']:.4f} | {r['cond_max']:.0f} | "
                     f"{r['ik_converged_pct']:.0f}% |"))
    return [f"{name}: expected '{w}'" for text, name, w in want if w not in text]


def check_paths():
    """Every path the scripts resolve must still point at something.

    Each script finds the repository by counting directories above itself, so
    moving the scripts silently redirects those paths one level off. Only the
    ones that read the URDF notice, and only when they are run, which is how a
    rename shipped with kinematics.py pointing outside the repository.
    """
    here = Path(__file__).resolve().parent
    sys.path.insert(0, str(here))
    bad = []
    try:
        import kinematics
        if not kinematics.URDF_XACRO.exists():
            bad.append(f"kinematics.URDF_XACRO does not exist: {kinematics.URDF_XACRO}")
    except Exception as e:                                    # pragma: no cover
        bad.append(f"kinematics.py will not import: {e}")
    for mod, attr in (("rospath", "ARTWORK"), ("analysis", "OUT"),
                      ("control_study", "OUT"), ("figures", "FIG")):
        try:
            m = __import__(mod)
            d = getattr(m, attr)
            if not d.parent.is_dir():
                bad.append(f"{mod}.{attr} is not inside the repository: {d}")
        except Exception as e:                                # pragma: no cover
            bad.append(f"{mod}.py will not import: {e}")
    return bad


def main():
    nums, links, paths = check_numbers(), check_links(), check_paths()
    traced, checked, declared = check_traceability()
    gains = check_gains()
    shares = check_shares()
    inertia = check_inertia_table()
    mass = check_mass()
    kin = check_kinematics()
    for b in nums + traced + gains + shares + inertia + mass + kin + links + paths:
        print("  FAIL", b)
    print(f"  {len(claims())} documented numbers, "
          f"{'all match' if not nums else str(len(nums)) + ' stale'}")
    print(f"  {checked} measured figures, "
          + (f"all in the data ({declared} declared)" if not traced
             else f"{len(traced)} untraceable"))
    print(f"  published gains:   {'match the data' if not gains else str(len(gains)) + ' stale'}")
    print("  torque shares:     "
          + ("match the data" if not shares else f"{len(shares)} stale"))
    print("  inertia table:     "
          + ("matches the data" if not inertia else f"{len(inertia)} stale"))
    print("  mass model:        "
          + ("matches the data" if not mass else f"{len(mass)} stale"))
    print("  kinematics:        "
          + ("matches the data" if not kin else f"{len(kin)} stale"))
    print(f"  links and anchors: {'all resolve' if not links else str(len(links)) + ' broken'}")
    print(f"  script paths:      {'all resolve' if not paths else str(len(paths)) + ' broken'}")
    failed = nums + traced + gains + shares + inertia + mass + kin + links + paths
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
