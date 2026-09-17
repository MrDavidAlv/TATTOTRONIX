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
Style, as ament defines it.

Scoped to this package. The stock template passes no path and lints whatever
the working directory happens to be, which under colcon is the whole workspace
- so a package's own test would fail over files in a sibling, or in docs. The
repository-wide check lives in tests/test_repo_style.py.
"""

from pathlib import Path

from ament_flake8.main import main_with_errors
import pytest

PACKAGE = Path(__file__).resolve().parents[1]


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    rc, errors = main_with_errors(argv=[str(PACKAGE)])
    assert rc == 0, \
        'Found %d code style errors / warnings:\n' % len(errors) + \
        '\n'.join(str(e) for e in errors)
