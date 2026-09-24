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


def test_a_grouped_figure_is_read_as_one_number():
    """'30 037 um' is thirty thousand, not thirty-seven.

    Read digit by digit it becomes "037", which rounds from 37.4 um elsewhere in
    the data and passes by coincidence - the gate agreeing with a number the
    document never wrote.
    """
    import check_docs
    for text in ('30 037 µm', '30 037 µm', '30 037 µm'):
        m = check_docs._FIGURE.search(text)
        assert m is not None and m.group(2) == 'µm', text
        assert check_docs._traceable(m.group(1), [30036.68]), text
        assert not check_docs._traceable(m.group(1), [37.4]), text


def test_a_figure_has_to_round_from_the_data_not_merely_resemble_it():
    """94.67 may be written 94.7 or 95. It may not be written 94."""
    import check_docs
    data = [94.67]
    assert check_docs._traceable('94.7', data)
    assert check_docs._traceable('95', data)
    assert not check_docs._traceable('94', data)
    assert not check_docs._traceable('94.6', data)


def test_every_declared_figure_says_why():
    """An exemption without a reason is just a number nobody checked."""
    import check_docs
    for (doc, figure), reason in check_docs.DECLARED.items():
        assert len(reason.split()) >= 5, f'{doc}: {figure} has no real reason'
