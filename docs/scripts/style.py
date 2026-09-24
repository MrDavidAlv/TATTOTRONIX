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
The project's visual style, shared by the documentation figures and the notebooks.

Kept apart from figures.py because that module selects the Agg backend when it
is imported, which is right for writing PNGs from a script and wrong inside a
notebook, where it would stop every plot from appearing. Importing this module
changes nothing until apply() is called.

Palette and mark specs follow the project's data-viz rules: categorical hues in
fixed order and never cycled, one hue light-to-dark for magnitude, thin marks,
recessive grid, and an explicit light surface so a figure reads the same in a
dark editor as in a browser.
"""

from matplotlib.colors import LinearSegmentedColormap

SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK2 = "#52514e"
MUTED = "#8a8984"
GRID = "#e4e3df"

# categorical, fixed order
C = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#4a3aa7", "#e34948"]
# sequential, one hue light -> dark
BLUES = ["#cde2fb", "#9ec5f4", "#6da7ec", "#3987e5", "#256abf", "#184f95", "#0d366b"]
SEQ = LinearSegmentedColormap.from_list("seq", BLUES)

RC = {
    "figure.facecolor": SURFACE, "axes.facecolor": SURFACE,
    "savefig.facecolor": SURFACE, "savefig.dpi": 200, "figure.dpi": 200,
    "font.family": "DejaVu Sans", "font.size": 9,
    "text.color": INK, "axes.labelcolor": INK2, "axes.titlecolor": INK,
    "xtick.color": INK2, "ytick.color": INK2,
    "axes.edgecolor": GRID, "axes.linewidth": 0.8,
    "grid.color": GRID, "grid.linewidth": 0.7,
    "lines.linewidth": 1.6, "lines.solid_capstyle": "round",
    "legend.frameon": False, "axes.spines.top": False, "axes.spines.right": False,
}


def apply(**overrides):
    """Set the project's rcParams, with any overrides on top."""
    import matplotlib.pyplot as plt
    plt.rcParams.update({**RC, **overrides})
