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
Expand the description once, for places that cannot run xacro.

Google Colab has no ROS, so it has no xacro and no package index to resolve
$(find tattotronix_description) against. The notebooks read this expansion
instead - the same URDF robot_state_publisher loads, with the default
arguments - and build the chain and the dynamic model from it with the same
code every study uses. tests/test_notebooks.py expands the xacro again and
fails if this file has fallen behind it.

    python3 docs/scripts/export_urdf.py
"""

import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
XACRO = ROOT / "src" / "tattotronix_description" / "urdf" / "tattotronix.urdf.xacro"
OUT = ROOT / "docs" / "notebooks" / "tattotronix.urdf"

#: The arguments the studies use: the pen mounted, no ros2_control block, and
#: the description's default mass model.
ARGS = ["tool:=tattoo", "hardware:=none"]


def expand():
    """The URDF text, exactly as xacro emits it for ARGS."""
    out = subprocess.run(["xacro", str(XACRO), *ARGS], capture_output=True, text=True)
    if out.returncode != 0:                                   # pragma: no cover
        sys.exit("xacro failed:\n" + out.stderr)
    return out.stdout


def main():
    OUT.write_text(expand(), encoding="utf-8")
    print("wrote", OUT.relative_to(ROOT))


if __name__ == "__main__":
    main()
