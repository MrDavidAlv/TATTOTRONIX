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
Style for the parts of the repository colcon never sees.

The ROS packages are linted by ament during colcon test, scoped to themselves.
Everything else - docs/scripts, tools, these tests - is linted here, against the
configuration in setup.cfg, where every exception is scoped and carries its
reason.
"""

import subprocess
import sys

LINTED = ['docs/scripts', 'tools', 'tests']


def test_flake8(repo):
    result = subprocess.run(
        [sys.executable, '-m', 'flake8', *LINTED],
        cwd=repo, capture_output=True, text=True)
    assert result.returncode == 0, \
        'style violations outside the ROS packages:\n' + result.stdout


def test_every_script_compiles(repo):
    """Catches a syntax error in a script nothing imports during a test run."""
    scripts = sorted((repo / 'docs' / 'scripts').glob('*.py'))
    assert scripts
    result = subprocess.run(
        [sys.executable, '-m', 'py_compile', *[str(p) for p in scripts]],
        cwd=repo, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr


def test_no_ai_attribution_in_the_history(repo):
    """The repository's own rule, enforced rather than remembered.

    Signing commits as an assistant takes credit for work that was several
    people's and several tools', and this project asks for none of it in the
    tree. It has had to be cleaned out of published history once.
    """
    result = subprocess.run(
        ['git', 'log', '--format=%an <%ae>%n%B', 'origin/main..HEAD'],
        cwd=repo, capture_output=True, text=True)
    if result.returncode != 0:          # no such ref in a shallow CI checkout
        result = subprocess.run(['git', 'log', '--format=%an <%ae>%n%B', '-50'],
                                cwd=repo, capture_output=True, text=True)
    banned = ('co-authored-by: claude', 'generated with', 'claude-session',
              'noreply@anthropic.com')
    found = [b for b in banned if b in result.stdout.lower()]
    assert not found, f'attribution found in commit history: {found}'
