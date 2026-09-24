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

"""Docstring conventions, scoped to this package for the same reason."""

from pathlib import Path

from ament_pep257.main import main
import pytest

PACKAGE = Path(__file__).resolve().parents[1]


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    assert main(argv=[str(PACKAGE)]) == 0, 'Found code style errors / warnings'
