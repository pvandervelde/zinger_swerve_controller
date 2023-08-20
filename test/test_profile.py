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

import math
import pytest
from typing import Mapping, List, Tuple

# locals
from zinger_swerve_controller.errors import InvalidTimeFractionException
from zinger_swerve_controller.profile import InvalidTimeFractionException, SingleVariableSCurveProfile

# SingleVariableSCurveProfile

def test_should_show_first_derivative_at_with_increasing_scurve_profile():
    start = 1.0
    end = 2.0
    profile = SingleVariableSCurveProfile(start, end)

    assert math.isclose(profile.first_derivative_at(0.0), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(1.0), 0.0, rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.first_derivative_at(1/8), 0.5 * 51.2 * 1/64, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(2/8), 1.5 * 51.2 * 1/64, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(3/8), 2.0 * 51.2 * 1/64, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(4/8), 2.0 * 51.2 * 1/64, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(5/8), 2.0 * 51.2 * 1/64, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(6/8), 1.5 * 51.2 * 1/64, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(7/8), 0.5 * 51.2 * 1/64, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(8/8), 0.0, rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.first_derivative_at(1/16), 0.5 * 51.2 * 1/256, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(3/16), 1.0 * 51.2 * 1/64, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(5/16), 51.2 * 1/128 - 0.5 * 51.2 * 1/256 + 1.5 * 51.2 * 1/64, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(7/16), 2.0 * 51.2 * 1/64, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(9/16), 2.0 * 51.2 * 1/64, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(11/16), 51.2 * 1/128 - 0.5 * 51.2 * 1/256 + 1.5 * 51.2 * 1/64, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(13/16), 1.0 * 51.2 * 1/64, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(15/16), 0.5 * 51.2 * 1/256, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_first_derivative_at_with_decreasing_scurve_profile():
    start = 2.0
    end = 1.0
    profile = SingleVariableSCurveProfile(start, end)

    assert math.isclose(profile.first_derivative_at(0.0), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(1.0), 0.0, rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.first_derivative_at(1/8), -0.5 * 51.2 * 1/64, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(2/8), -1.5 * 51.2 * 1/64, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(3/8), -2.0 * 51.2 * 1/64, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(4/8), -2.0 * 51.2 * 1/64, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(5/8), -2.0 * 51.2 * 1/64, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(6/8), -1.5 * 51.2 * 1/64, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(7/8), -0.5 * 51.2 * 1/64, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(8/8), 0.0, rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.first_derivative_at(1/16), -0.5 * 51.2 * 1/256, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(3/16), -1.0 * 51.2 * 1/64, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(5/16), -51.2 * 1/128 + 0.5 * 51.2 * 1/256 - 1.5 * 51.2 * 1/64, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(7/16), -2.0 * 51.2 * 1/64, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(9/16), -2.0 * 51.2 * 1/64, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(11/16), -51.2 * 1/128 + 0.5 * 51.2 * 1/256 - 1.5 * 51.2 * 1/64, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(13/16), -1.0 * 51.2 * 1/64, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(15/16), -0.5 * 51.2 * 1/256, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_second_derivative_at_with_increasing_scurve_profile():
    start = 1.0
    end = 2.0
    profile = SingleVariableSCurveProfile(start, end)

    assert math.isclose(profile.second_derivative_at(0.0), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(1.0), 0.0, rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.second_derivative_at(1/8), 51.2 * 1/8, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(2/8), 51.2 * 1/8, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(3/8), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(4/8), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(5/8), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(6/8), -51.2 * 1/8, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(7/8), -51.2 * 1/8, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(8/8), 0.0, rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.second_derivative_at(1/16), 51.2 * 1/16, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(3/16), 51.2 * 1/8, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(5/16), 51.2 * 1/16, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(7/16), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(9/16), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(11/16), -51.2 * 1/16, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(13/16), -51.2 * 1/8, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(15/16), -51.2 * 1/16, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_second_derivative_at_with_decreasing_scurve_profile():
    start = 2.0
    end = 1.0
    profile = SingleVariableSCurveProfile(start, end)

    assert math.isclose(profile.second_derivative_at(0.0), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(1.0), 0.0, rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.second_derivative_at(1/8), -51.2 * 1/8, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(2/8), -51.2 * 1/8, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(3/8), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(4/8), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(5/8), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(6/8), 51.2 * 1/8, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(7/8), 51.2 * 1/8, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(8/8), 0.0, rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.second_derivative_at(1/16), -51.2 * 1/16, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(3/16), -51.2 * 1/8, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(5/16), -51.2 * 1/16, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(7/16), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(9/16), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(11/16), 51.2 * 1/16, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(13/16), 51.2 * 1/8, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(15/16), 51.2 * 1/16, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_third_derivative_at_with_increasing_scurve_profile():
    start = 1.0
    end = 2.0
    profile = SingleVariableSCurveProfile(start, end)

    assert math.isclose(profile.third_derivative_at(0.0), 51.2, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(1.0), 51.2, rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.third_derivative_at(1/8), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(2/8), -51.2, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(3/8), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(4/8), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(5/8), -51.2, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(6/8), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(7/8), 51.2, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(8/8), 51.2, rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.third_derivative_at(1/16), 51.2, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(3/16), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(5/16), -51.2, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(7/16), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(9/16), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(11/16), -51.2, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(13/16), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(15/16), 51.2, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_third_derivative_at_with_decreasing_scurve_profile():
    start = 2.0
    end = 1.0
    profile = SingleVariableSCurveProfile(start, end)

    assert math.isclose(profile.third_derivative_at(0.0), -51.2, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(1.0), -51.2, rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.third_derivative_at(1/8), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(2/8), 51.2, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(3/8), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(4/8), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(5/8), 51.2, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(6/8), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(7/8), -51.2, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(8/8), -51.2, rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.third_derivative_at(1/16), -51.2, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(3/16), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(5/16), 51.2, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(7/16), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(9/16), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(11/16), 51.2, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(13/16), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(15/16), -51.2, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_value_at_with_increasing_scurve_profile():
    start = 1.0
    end = 2.0
    profile = SingleVariableSCurveProfile(start, end)

    assert math.isclose(profile.value_at(0.0), start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(1.0), end, rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.value_at(1/8), 1/6 * 51.2 * 1/512 + start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(2/8), 1.0 * 51.2 * 1/512 + 1/6 * 51.2 * 1/512 + start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(3/8), 3.0 * 51.2 * 1/512 + start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(4/8), 5.0 * 51.2 * 1/512 + start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(5/8), 7.0 * 51.2 * 1/512 + start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(6/8), (8 + 5/6) * 51.2 * 1/512 + start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(7/8), (9 + 5/6) * 51.2 * 1/512 + start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(8/8), end, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_value_at_with_decreasing_scurve_profile():
    start = 2.0
    end = 1.0
    profile = SingleVariableSCurveProfile(start, end)

    assert math.isclose(profile.value_at(0.0), start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(1.0), end, rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.value_at(1/8), -1/6 * 51.2 * 1/512 + start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(2/8), -1.0 * 51.2 * 1/512 - 1/6 * 51.2 * 1/512 + start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(3/8), -3.0 * 51.2 * 1/512 + start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(4/8), -5.0 * 51.2 * 1/512 + start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(5/8), -7.0 * 51.2 * 1/512 + start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(6/8), -(8 + 5/6) * 51.2 * 1/512 + start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(7/8), -(9 + 5/6) * 51.2 * 1/512 + start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(8/8), end, rel_tol=1e-6, abs_tol=1e-15)
