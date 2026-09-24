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

"""
Build the notebooks from their sources.

Each notebook is written as a plain Python file in the "percent" format -
"# %%" starts a code cell, "# %% [markdown]" a text cell whose lines are
comments - under docs/notebooks/src. Sources diff and review like code; the
.ipynb files are generated from them and never edited by hand.

Two things are added on the way. The Colab setup cell, identical in every
notebook, replaces the "# %% [setup]" marker: it clones the repository when it
runs in Colab and finds it everywhere else. And an "Open in Colab" badge goes
under the first heading.

tests/test_notebooks.py rebuilds every notebook and compares, then runs the
code cells of the built notebook - not the source - so what is certified is
what a reader opens.

    python3 docs/notebooks/build.py                       # build every notebook
    python3 docs/notebooks/build.py --run 01_kinematics   # run one, headless
    python3 docs/notebooks/build.py --export 01_kinematics  # run it, writing figures
"""

import argparse
import json
import os
import re
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
SRC = HERE / "src"
REPO_URL = "https://github.com/MrDavidAlv/TATTOTRONIX"
BRANCH = "humble"

SETUP = '''# Setup. In Google Colab this clones the repository, which carries the robot's
# description, meshes and model code; anywhere else it finds the checkout it is in.
import os
import sys
from pathlib import Path

BRANCH = "{branch}"
if "google.colab" in sys.modules and not Path("/content/TATTOTRONIX").is_dir():
    import subprocess
    subprocess.run(["git", "clone", "--depth", "1", "--branch", BRANCH,
                    "{url}.git",
                    "/content/TATTOTRONIX"], check=True)
    os.chdir("/content/TATTOTRONIX/docs/notebooks")
for _p in [Path.cwd(), *Path.cwd().parents, Path("/content/TATTOTRONIX")]:
    if (_p / "docs" / "notebooks" / "nbsupport.py").is_file():
        sys.path.insert(0, str(_p / "docs" / "notebooks"))
        break

import nbsupport as nb  # noqa: E402
print("repository:", nb.REPO)'''.format(branch=BRANCH, url=REPO_URL)


def badge(name):
    url = (f"https://colab.research.google.com/github/MrDavidAlv/TATTOTRONIX/blob/{BRANCH}"
           f"/docs/notebooks/{name}.ipynb")
    return f"[![Open in Colab](https://colab.research.google.com/assets/colab-badge.svg)]({url})"


def cells_of(text):
    """Split percent-format source into (kind, lines) cells."""
    cells, kind, lines = [], None, []
    for line in text.splitlines():
        m = re.match(r"^# %%(?:\s*\[(markdown|setup)\])?\s*$", line)
        if m:
            if kind is not None:
                cells.append((kind, lines))
            kind, lines = (m.group(1) or "code"), []
        elif kind is not None:
            lines.append(line)
    if kind is not None:
        cells.append((kind, lines))
    return cells


def _source(lines):
    while lines and not lines[-1].strip():
        lines = lines[:-1]
    while lines and not lines[0].strip():
        lines = lines[1:]
    return [line + "\n" for line in lines[:-1]] + lines[-1:]


def build(path):
    """The notebook JSON for one source file."""
    name = path.stem
    out, badged = [], False
    for i, (kind, lines) in enumerate(cells_of(path.read_text(encoding="utf-8"))):
        cid = f"{name}-{i:02d}"
        if kind == "markdown":
            text = [re.sub(r"^# ?", "", ln) for ln in lines]
            if not badged and text and text[0].startswith("# "):
                text = text[:1] + ["", badge(name)] + text[1:]
                badged = True
            out.append({"cell_type": "markdown", "id": cid, "metadata": {},
                        "source": _source(text)})
        else:
            code = SETUP.splitlines() if kind == "setup" else lines
            out.append({"cell_type": "code", "id": cid, "metadata": {},
                        "execution_count": None, "outputs": [], "source": _source(code)})
    nb = {
        "cells": out,
        "metadata": {
            "colab": {"provenance": [], "toc_visible": True},
            "kernelspec": {"display_name": "Python 3", "language": "python", "name": "python3"},
            "language_info": {"name": "python"},
        },
        "nbformat": 4,
        "nbformat_minor": 5,
    }
    return json.dumps(nb, indent=1, ensure_ascii=False) + "\n"


def sources():
    return sorted(SRC.glob("*.py"))


def code_of(name):
    """Every code cell of a built notebook, in order, as one script."""
    cells = json.loads((HERE / f"{name}.ipynb").read_text(encoding="utf-8"))["cells"]
    return "\n\n\n".join("".join(c["source"]) for c in cells if c["cell_type"] == "code")


def run(name, export=False, timeout=1800):
    """Run a built notebook headless, as a reader would, from docs/notebooks.

    With export, the figures it names are written to docs/figures.
    """
    env = {**os.environ, "MPLBACKEND": "Agg"}
    env.pop("TATTO_EXPORT", None)
    if export:
        env["TATTO_EXPORT"] = "1"
    return subprocess.run([sys.executable, "-c", code_of(name)], cwd=HERE, env=env,
                          capture_output=True, text=True, timeout=timeout)


def main():
    ap = argparse.ArgumentParser(description="Build, run or export the notebooks.")
    ap.add_argument("--run", nargs="+", metavar="NAME", help="run built notebooks")
    ap.add_argument("--export", nargs="+", metavar="NAME",
                    help="run built notebooks and write their figures to docs/figures")
    args = ap.parse_args()
    for name, export in [(n, False) for n in args.run or []] + \
                        [(n, True) for n in args.export or []]:
        out = run(name, export=export)
        print(out.stdout, end="")
        if out.returncode != 0:
            sys.exit(out.stderr)
    if args.run or args.export:
        return
    for src in sources():
        target = HERE / (src.stem + ".ipynb")
        target.write_text(build(src), encoding="utf-8")
        print("wrote", target.relative_to(HERE.parents[1]))


if __name__ == "__main__":
    main()
