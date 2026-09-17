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
The toolpath, and the contour tracer's own certification.

rospath.selftest() checks the tracer against shapes whose topology is known.
Running it from pytest is what makes it part of the build rather than something
someone remembers to run.
"""

import numpy as np
import pytest

import rospath as rp


def test_contour_tracer_self_certification():
    assert rp.selftest(verbose=False), 'the contour tracer failed its own checks'


def test_stroke_pitch_is_declared_against_the_line_width():
    """The fill pitch is not coverage, and the gap should be visible in code.

    A 0.3 mm needle at a 1.2 mm pitch leaves 0.9 mm of skin between passes.
    That is hatching, and it is a legitimate choice, but it is not filling.
    """
    assert rp.STROKE_PITCH > 0
    assert rp.POINT_STEP > 0
    assert rp.POINT_STEP < rp.STROKE_PITCH, \
        'resampling coarser than the fill pitch samples the fill unevenly'


def test_placeholder_is_still_labelled_a_placeholder():
    """It stood in for the logo for a long time and hid two real defects."""
    assert 'NOT the ROS logo' in rp.placeholder_mask.__doc__ \
        or 'stand-in' in rp.placeholder_mask.__doc__.lower()


@pytest.mark.parametrize('method', ['threshold', 'edges'])
def test_both_mask_methods_produce_a_usable_mask(method):
    """A synthetic image, so the test needs no artwork it is not allowed to ship."""
    from PIL import Image
    import io
    img = np.full((200, 300), 255, np.uint8)
    img[60:140, 80:220] = 0
    buf = io.BytesIO()
    Image.fromarray(img).save(buf, format='PNG')
    buf.seek(0)
    mask, (xs, ys) = rp.from_image(buf, 100.0, method=method)
    assert mask.any(), f'{method} produced an empty mask'
    assert not mask.all(), f'{method} marked the entire panel as ink'
    assert abs((xs[-1] - xs[0]) - 100.0) < 1.0


def test_ink_is_the_minority_whatever_the_polarity():
    """Artwork on a black field must not come out as a filled rectangle."""
    from PIL import Image
    import io
    for background, foreground in ((255, 0), (0, 255)):
        img = np.full((200, 300), background, np.uint8)
        img[60:140, 80:220] = foreground
        buf = io.BytesIO()
        Image.fromarray(img).save(buf, format='PNG')
        buf.seek(0)
        mask, _ = rp.from_image(buf, 100.0)
        assert mask.mean() < 0.5, \
            f'ink covers {mask.mean() * 100:.0f}% on a {background} background'


def test_toolpath_never_marks_above_the_surface():
    mask, grid = rp.placeholder_mask()
    P, kind = rp.toolpath(mask, grid)
    marking = (kind[:-1] == rp.KIND_MARK) & (kind[1:] == rp.KIND_MARK)
    if marking.any():
        assert P[:-1][marking][:, 2].max() <= 0.0 + 1e-9
