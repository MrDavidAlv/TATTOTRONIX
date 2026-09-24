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
The notebooks, built from their sources and run end to end.

A notebook that only renders on the author's machine is not documentation, and
one whose asserts are never executed certifies nothing. So every notebook is
rebuilt from its source and compared, and then the code cells of the built
notebook - exactly what a reader opens in Colab - are run as one script,
headless. Each notebook checks its own mathematics with asserts; running it
here is what makes those checks part of the build.
"""

import subprocess
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
NB = REPO / "docs" / "notebooks"
sys.path.insert(0, str(NB))

import build  # noqa: E402

SOURCES = build.sources()


def test_the_expanded_urdfs_are_current():
    """The URDFs the notebooks read have to be the description as it stands."""
    sys.path.insert(0, str(REPO / "docs" / "scripts"))
    import export_urdf
    for model, path in export_urdf.FILES.items():
        assert path.read_text(encoding="utf-8") == export_urdf.expand(model), (
            f"{path.name} is stale. Run: python3 docs/scripts/export_urdf.py")


def test_there_are_notebooks():
    assert SOURCES, "no notebook sources under docs/notebooks/src"


@pytest.mark.parametrize("src", SOURCES, ids=lambda p: p.stem)
def test_the_notebook_is_built_from_its_source(src):
    built = NB / (src.stem + ".ipynb")
    assert built.is_file(), f"{built.name} has not been built"
    assert built.read_text(encoding="utf-8") == build.build(src), (
        f"{built.name} is stale. Run: python3 docs/notebooks/build.py")


@pytest.mark.parametrize("src", SOURCES, ids=lambda p: p.stem)
def test_the_notebook_is_valid(src):
    nbformat = pytest.importorskip("nbformat")
    nbformat.validate(nbformat.reads((NB / (src.stem + ".ipynb")).read_text(), as_version=4))


@pytest.mark.parametrize("src", SOURCES, ids=lambda p: p.stem)
def test_the_notebook_runs(src):
    """Every code cell, in order, as one script - asserts included."""
    out = build.run(src.stem, timeout=900)
    assert out.returncode == 0, out.stdout[-3000:] + out.stderr[-3000:]


@pytest.mark.parametrize("src", SOURCES, ids=lambda p: p.stem)
def test_the_notebook_code_is_clean(src, tmp_path):
    """flake8 over the code a reader runs: setup cell included, markdown removed.

    E402 is expected - every notebook imports from the repository only after its
    setup cell has put it on the path. Nothing else is excused, and in particular
    not F821, which is what catches an import a cell forgot.
    """
    script = tmp_path / (src.stem + ".py")
    script.write_text(build.code_of(src.stem) + "\n", encoding="utf-8")
    out = subprocess.run([sys.executable, "-m", "flake8", "--isolated", "--max-line-length=99",
                          "--extend-ignore=E402", str(script)], capture_output=True, text=True)
    assert out.returncode == 0, out.stdout
