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
The documents, checked against the data they quote.

check_docs.py is the certification the repository runs on: it fails if a
published number no longer matches the file it came from, if a link or image is
broken, or if a script's idea of where the repository is has drifted. Running it
from pytest puts it in the build.
"""

import subprocess
import sys


def test_documents_match_the_data(repo):
    result = subprocess.run(
        [sys.executable, 'docs/scripts/check_docs.py'],
        cwd=repo, capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr
