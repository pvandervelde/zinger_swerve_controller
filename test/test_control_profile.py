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
from zinger_swerve_controller.control_model import DriveModuleDesiredValues, DriveModuleMeasuredValues, BodyMotion, SimpleFourWheelSteeringControlModel
from zinger_swerve_controller.control_profile import BodyMotionProfile, DriveModuleStateProfile
from zinger_swerve_controller.drive_module import DriveModule
from zinger_swerve_controller.errors import IncompleteTrajectoryException
from zinger_swerve_controller.geometry import Point
from zinger_swerve_controller.states import BodyState

def create_drive_modules(
    length: float = 1.0,
    width: float = 1.0,
    wheel_radius: float = 0.1,
    wheel_width: float = 0.05,
    steering_max_velocity: float = 1.0,
    steering_min_acceleration: float = 0.1,
    steering_max_acceleration: float = 1.0,
    drive_max_velocity: float = 1.0,
    drive_min_acceleration: float = 0.1,
    drive_max_acceleration: float = 1.0) -> List[DriveModule]:
    result: List[DriveModule] = []

    right_front_drive = DriveModule(
        name="module_1",
        steering_link="steering_link_1",
        drive_link="drive_link_1",
        steering_axis_xy_position=Point(0.5 * length, -0.5 * width, 0.0),
        wheel_radius=wheel_radius,
        wheel_width=wheel_width,
        steering_motor_maximum_velocity=steering_max_velocity,
        steering_motor_minimum_acceleration=steering_min_acceleration,
        steering_motor_maximum_acceleration=steering_max_acceleration,
        drive_motor_maximum_velocity=drive_max_velocity,
        drive_motor_minimum_acceleration=drive_min_acceleration,
        drive_motor_maximum_acceleration=drive_max_acceleration
    )
    result.append(right_front_drive)

    left_front_drive = DriveModule(
        name="module_2",
        steering_link="steering_link_2",
        drive_link="drive_link_2",
        steering_axis_xy_position=Point(0.5 * length, 0.5 * width, 0.0),
        wheel_radius=wheel_radius,
        wheel_width=wheel_width,
        steering_motor_maximum_velocity=steering_max_velocity,
        steering_motor_minimum_acceleration=steering_min_acceleration,
        steering_motor_maximum_acceleration=steering_max_acceleration,
        drive_motor_maximum_velocity=drive_max_velocity,
        drive_motor_minimum_acceleration=drive_min_acceleration,
        drive_motor_maximum_acceleration=drive_max_acceleration
    )
    result.append(left_front_drive)

    left_rear_drive = DriveModule(
        name="module_3",
        steering_link="steering_link_3",
        drive_link="drive_link_3",
        steering_axis_xy_position=Point(-0.5 * length, 0.5 * width, 0.0),
        wheel_radius=wheel_radius,
        wheel_width=wheel_width,
        steering_motor_maximum_velocity=steering_max_velocity,
        steering_motor_minimum_acceleration=steering_min_acceleration,
        steering_motor_maximum_acceleration=steering_max_acceleration,
        drive_motor_maximum_velocity=drive_max_velocity,
        drive_motor_minimum_acceleration=drive_min_acceleration,
        drive_motor_maximum_acceleration=drive_max_acceleration
    )
    result.append(left_rear_drive)

    right_rear_drive = DriveModule(
        name="module_4",
        steering_link="steering_link_4",
        drive_link="drive_link_4",
        steering_axis_xy_position=Point(-0.5 * length, -0.5 * width, 0.0),
        wheel_radius=wheel_radius,
        wheel_width=wheel_width,
        steering_motor_maximum_velocity=steering_max_velocity,
        steering_motor_minimum_acceleration=steering_min_acceleration,
        steering_motor_maximum_acceleration=steering_max_acceleration,
        drive_motor_maximum_velocity=drive_max_velocity,
        drive_motor_minimum_acceleration=drive_min_acceleration,
        drive_motor_maximum_acceleration=drive_max_acceleration
    )
    result.append(right_rear_drive)

    return result

# BodyMotionTrajectory

def test_should_show_value_at_in_body_motion():
    time = 10.0

    start_motion = BodyState(
        0.0,
        0.0,
        0.0,
        1.0,
        2.0,
        3.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )
    end_motion = BodyMotion(
        4.0,
        5.0,
        6.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )

    trajectory = BodyMotionProfile(start_motion, end_motion, time)

    assert math.isclose(trajectory.body_motion_at(0.0).linear_velocity.x, 1.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(trajectory.body_motion_at(0.0).linear_velocity.y, 2.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(trajectory.body_motion_at(0.0).angular_velocity.z, 3.0, rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(trajectory.body_motion_at(1.0).linear_velocity.x, 4.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(trajectory.body_motion_at(1.0).linear_velocity.y, 5.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(trajectory.body_motion_at(1.0).angular_velocity.z, 6.0, rel_tol=1e-6, abs_tol=1e-15)

# DriveModuleStateTrajectory

def test_drive_module_trajectory_should_create_trajectory_with_start():
    drive_modules = create_drive_modules()

    trajectory = DriveModuleStateProfile(drive_modules, 1.0)

    current_states: List[DriveModuleMeasuredValues] = []
    for i in range(len(drive_modules)):
        module_state = DriveModuleMeasuredValues(
            drive_modules[i].name,
            drive_modules[i].steering_axis_xy_position.x,
            drive_modules[i].steering_axis_xy_position.y,
            math.radians(0),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )
        current_states.append(module_state)
    trajectory.set_current_state(current_states)

    with pytest.raises(IncompleteTrajectoryException):
        trajectory.align_module_profiles()

def test_drive_module_trajectory_should_create_trajectory_with_end():
    drive_modules = create_drive_modules()

    trajectory = DriveModuleStateProfile(drive_modules, 1.0)

    states: List[DriveModuleMeasuredValues] = []
    for i in range(len(drive_modules)):
        module_state = DriveModuleMeasuredValues(
            drive_modules[i].name,
            drive_modules[i].steering_axis_xy_position.x,
            drive_modules[i].steering_axis_xy_position.y,
            math.radians(0),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )
        states.append(module_state)
    trajectory.set_desired_end_state(states)

    with pytest.raises(IncompleteTrajectoryException):
        trajectory.align_module_profiles()

def test_drive_module_trajectory_should_create_trajectory_for_forward_acceleration():
    drive_modules = create_drive_modules()

    trajectory = DriveModuleStateProfile(drive_modules, 1.0)

    current_states: List[DriveModuleMeasuredValues] = []
    for i in range(len(drive_modules)):
        module_state = DriveModuleMeasuredValues(
            drive_modules[i].name,
            drive_modules[i].steering_axis_xy_position.x,
            drive_modules[i].steering_axis_xy_position.y,
            math.radians(0),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )
        current_states.append(module_state)
    trajectory.set_current_state(current_states)

    desired_states: List[DriveModuleDesiredValues] = []
    for i in range(len(drive_modules)):
        module_state = DriveModuleDesiredValues(
            drive_modules[i].name,
            math.radians(0),
            1.0,
        )
        desired_states.append(module_state)
    trajectory.set_desired_end_state(desired_states)
    trajectory.align_module_profiles()

    for drive_module in drive_modules:

        state = trajectory.value_for_module_at(drive_module.name, 0.0)
        assert math.isclose(state.drive_acceleration_in_module_coordinates.x, 1.0, rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.drive_velocity_in_module_coordinates.x, 0.0, rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.orientation_velocity_in_body_coordinates.z, 0.0, rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.orientation_in_body_coordinates.z, 0.0, rel_tol=1e-6, abs_tol=1e-6)

        state = trajectory.value_for_module_at(drive_module.name, 0.5)
        assert math.isclose(state.drive_acceleration_in_module_coordinates.x, 1.0, rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.drive_velocity_in_module_coordinates.x, 0.5, rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.orientation_velocity_in_body_coordinates.z, 0.0, rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.orientation_in_body_coordinates.z, 0.0, rel_tol=1e-6, abs_tol=1e-6)

        state = trajectory.value_for_module_at(drive_module.name, 1.0)
        assert math.isclose(state.drive_acceleration_in_module_coordinates.x, 1.0, rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.drive_velocity_in_module_coordinates.x, 1.0, rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.orientation_velocity_in_body_coordinates.z, 0.0, rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.orientation_in_body_coordinates.z, 0.0, rel_tol=1e-6, abs_tol=1e-6)

def test_drive_module_trajectory_should_create_trajectory_for_forward_deceleration():
    drive_modules = create_drive_modules()

    trajectory = DriveModuleStateProfile(drive_modules, 1.0)

    current_states: List[DriveModuleMeasuredValues] = []
    for i in range(len(drive_modules)):
        module_state = DriveModuleMeasuredValues(
            drive_modules[i].name,
            drive_modules[i].steering_axis_xy_position.x,
            drive_modules[i].steering_axis_xy_position.y,
            math.radians(0),
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
            0.0,
        )
        current_states.append(module_state)
    trajectory.set_current_state(current_states)

    desired_states: List[DriveModuleMeasuredValues] = []
    for i in range(len(drive_modules)):
        module_state = DriveModuleDesiredValues(
            drive_modules[i].name,
            math.radians(0),
            0.0,
        )
        desired_states.append(module_state)
    trajectory.set_desired_end_state(desired_states)
    trajectory.align_module_profiles()

    for drive_module in drive_modules:

        state = trajectory.value_for_module_at(drive_module.name, 0.0)
        assert math.isclose(state.drive_acceleration_in_module_coordinates.x, -1.0, rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.drive_velocity_in_module_coordinates.x, 1.0, rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.orientation_velocity_in_body_coordinates.z, 0.0, rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.orientation_in_body_coordinates.z, 0.0, rel_tol=1e-6, abs_tol=1e-6)

        state = trajectory.value_for_module_at(drive_module.name, 0.5)
        assert math.isclose(state.drive_acceleration_in_module_coordinates.x, -1.0, rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.drive_velocity_in_module_coordinates.x, 0.5, rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.orientation_velocity_in_body_coordinates.z, 0.0, rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.orientation_in_body_coordinates.z, 0.0, rel_tol=1e-6, abs_tol=1e-6)

        state = trajectory.value_for_module_at(drive_module.name, 1.0)
        assert math.isclose(state.drive_acceleration_in_module_coordinates.x, -1.0, rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.drive_velocity_in_module_coordinates.x, 0.0, rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.orientation_velocity_in_body_coordinates.z, 0.0, rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.orientation_in_body_coordinates.z, 0.0, rel_tol=1e-6, abs_tol=1e-6)

def test_drive_module_trajectory_should_create_trajectory_for_sideways_acceleration():
    drive_modules = create_drive_modules()

    trajectory = DriveModuleStateProfile(drive_modules, 1.0)

    current_states: List[DriveModuleMeasuredValues] = []
    for i in range(len(drive_modules)):
        module_state = DriveModuleMeasuredValues(
            drive_modules[i].name,
            drive_modules[i].steering_axis_xy_position.x,
            drive_modules[i].steering_axis_xy_position.y,
            math.radians(90),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )
        current_states.append(module_state)
    trajectory.set_current_state(current_states)

    desired_states: List[DriveModuleMeasuredValues] = []
    for i in range(len(drive_modules)):
        module_state = DriveModuleDesiredValues(
            drive_modules[i].name,
            math.radians(90),
            1.0,
        )
        desired_states.append(module_state)
    trajectory.set_desired_end_state(desired_states)
    trajectory.align_module_profiles()

    for drive_module in drive_modules:

        state = trajectory.value_for_module_at(drive_module.name, 0.0)
        assert math.isclose(state.drive_acceleration_in_module_coordinates.x, 1.0, rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.drive_velocity_in_module_coordinates.x, 0.0, rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.orientation_velocity_in_body_coordinates.z, 0.0, rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.orientation_in_body_coordinates.z, math.radians(90), rel_tol=1e-6, abs_tol=1e-6)

        state = trajectory.value_for_module_at(drive_module.name, 0.5)
        assert math.isclose(state.drive_acceleration_in_module_coordinates.x, 1.0, rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.drive_velocity_in_module_coordinates.x, 0.5, rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.orientation_velocity_in_body_coordinates.z, 0.0, rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.orientation_in_body_coordinates.z, math.radians(90), rel_tol=1e-6, abs_tol=1e-6)

        state = trajectory.value_for_module_at(drive_module.name, 1.0)
        assert math.isclose(state.drive_acceleration_in_module_coordinates.x, 1.0, rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.drive_velocity_in_module_coordinates.x, 1.0, rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.orientation_velocity_in_body_coordinates.z, 0.0, rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.orientation_in_body_coordinates.z, math.radians(90), rel_tol=1e-6, abs_tol=1e-6)

def test_drive_module_trajectory_should_create_trajectory_for_sideways_deceleration():
    drive_modules = create_drive_modules()

    trajectory = DriveModuleStateProfile(drive_modules, 1.0)

    current_states: List[DriveModuleMeasuredValues] = []
    for i in range(len(drive_modules)):
        module_state = DriveModuleMeasuredValues(
            drive_modules[i].name,
            drive_modules[i].steering_axis_xy_position.x,
            drive_modules[i].steering_axis_xy_position.y,
            math.radians(90),
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
            0.0,
        )
        current_states.append(module_state)
    trajectory.set_current_state(current_states)

    desired_states: List[DriveModuleMeasuredValues] = []
    for i in range(len(drive_modules)):
        module_state = DriveModuleDesiredValues(
            drive_modules[i].name,
            math.radians(90),
            0.0,
        )
        desired_states.append(module_state)
    trajectory.set_desired_end_state(desired_states)
    trajectory.align_module_profiles()

    for drive_module in drive_modules:

        state = trajectory.value_for_module_at(drive_module.name, 0.0)
        assert math.isclose(state.drive_acceleration_in_module_coordinates.x, -1.0, rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.drive_velocity_in_module_coordinates.x, 1.0, rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.orientation_velocity_in_body_coordinates.z, 0.0, rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.orientation_in_body_coordinates.z, math.radians(90), rel_tol=1e-6, abs_tol=1e-6)

        state = trajectory.value_for_module_at(drive_module.name, 0.5)
        assert math.isclose(state.drive_acceleration_in_module_coordinates.x, -1.0, rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.drive_velocity_in_module_coordinates.x, 0.5, rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.orientation_velocity_in_body_coordinates.z, 0.0, rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.orientation_in_body_coordinates.z, math.radians(90), rel_tol=1e-6, abs_tol=1e-6)

        state = trajectory.value_for_module_at(drive_module.name, 1.0)
        assert math.isclose(state.drive_acceleration_in_module_coordinates.x, -1.0, rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.drive_velocity_in_module_coordinates.x, 0.0, rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.orientation_velocity_in_body_coordinates.z, 0.0, rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.orientation_in_body_coordinates.z, math.radians(90), rel_tol=1e-6, abs_tol=1e-6)

def test_drive_module_trajectory_should_create_trajectory_for_rotational_acceleration():
    drive_modules = create_drive_modules()

    trajectory = DriveModuleStateProfile(drive_modules, 1.0)

    current_states: List[DriveModuleMeasuredValues] = []
    for i in range(len(drive_modules)):
        module_state = DriveModuleMeasuredValues(
            drive_modules[i].name,
            drive_modules[i].steering_axis_xy_position.x,
            drive_modules[i].steering_axis_xy_position.y,
            math.radians(90),
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
            0.0,
        )
        current_states.append(module_state)
    trajectory.set_current_state(current_states)

    desired_states: List[DriveModuleMeasuredValues] = []
    for i in range(len(drive_modules)):
        module_state = DriveModuleDesiredValues(
            drive_modules[i].name,
            math.radians(0),
            1.0,
        )
        desired_states.append(module_state)
    trajectory.set_desired_end_state(desired_states)
    trajectory.align_module_profiles()

    for drive_module in drive_modules:

        state = trajectory.value_for_module_at(drive_module.name, 0.0)
        assert math.isclose(state.drive_acceleration_in_module_coordinates.x, 0.0, rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.drive_velocity_in_module_coordinates.x, 1.0, rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.orientation_velocity_in_body_coordinates.z, -math.radians(90), rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.orientation_in_body_coordinates.z, math.radians(90), rel_tol=1e-6, abs_tol=1e-6)

        state = trajectory.value_for_module_at(drive_module.name, 0.5)
        assert math.isclose(state.drive_acceleration_in_module_coordinates.x, 0.0, rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.drive_velocity_in_module_coordinates.x, 1.0, rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.orientation_velocity_in_body_coordinates.z, -math.radians(90), rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.orientation_in_body_coordinates.z, math.radians(45), rel_tol=1e-6, abs_tol=1e-6)

        state = trajectory.value_for_module_at(drive_module.name, 1.0)
        assert math.isclose(state.drive_acceleration_in_module_coordinates.x, 0.0, rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.drive_velocity_in_module_coordinates.x, 1.0, rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.orientation_velocity_in_body_coordinates.z, -math.radians(90), rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.orientation_in_body_coordinates.z, math.radians(0), rel_tol=1e-6, abs_tol=1e-6)

def test_drive_module_trajectory_should_create_trajectory_for_rotational_deceleration():
    drive_modules = create_drive_modules()

    trajectory = DriveModuleStateProfile(drive_modules, 1.0)

    current_states: List[DriveModuleMeasuredValues] = []
    for i in range(len(drive_modules)):
        module_state = DriveModuleMeasuredValues(
            drive_modules[i].name,
            drive_modules[i].steering_axis_xy_position.x,
            drive_modules[i].steering_axis_xy_position.y,
            math.radians(0),
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
            0.0,
        )
        current_states.append(module_state)
    trajectory.set_current_state(current_states)

    desired_states: List[DriveModuleMeasuredValues] = []
    for i in range(len(drive_modules)):
        module_state = DriveModuleDesiredValues(
            drive_modules[i].name,
            math.radians(90),
            1.0,
        )
        desired_states.append(module_state)
    trajectory.set_desired_end_state(desired_states)
    trajectory.align_module_profiles()

    for drive_module in drive_modules:

        state = trajectory.value_for_module_at(drive_module.name, 0.0)
        assert math.isclose(state.drive_acceleration_in_module_coordinates.x, 0.0, rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.drive_velocity_in_module_coordinates.x, 1.0, rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.orientation_velocity_in_body_coordinates.z, math.radians(90), rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.orientation_in_body_coordinates.z, math.radians(0), rel_tol=1e-6, abs_tol=1e-6)

        state = trajectory.value_for_module_at(drive_module.name, 0.5)
        assert math.isclose(state.drive_acceleration_in_module_coordinates.x, 0.0, rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.drive_velocity_in_module_coordinates.x, 1.0, rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.orientation_velocity_in_body_coordinates.z, math.radians(90), rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.orientation_in_body_coordinates.z, math.radians(45), rel_tol=1e-6, abs_tol=1e-6)

        state = trajectory.value_for_module_at(drive_module.name, 1.0)
        assert math.isclose(state.drive_acceleration_in_module_coordinates.x, 0.0, rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.drive_velocity_in_module_coordinates.x, 1.0, rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.orientation_velocity_in_body_coordinates.z, math.radians(90), rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.orientation_in_body_coordinates.z, math.radians(90), rel_tol=1e-6, abs_tol=1e-6)

def test_drive_module_trajectory_should_create_trajectory_for_forwards_to_sideways_transition():
    drive_modules = create_drive_modules()

    trajectory = DriveModuleStateProfile(drive_modules, 1.0)

    current_states: List[DriveModuleMeasuredValues] = []
    for i in range(len(drive_modules)):
        module_state = DriveModuleMeasuredValues(
            drive_modules[i].name,
            drive_modules[i].steering_axis_xy_position.x,
            drive_modules[i].steering_axis_xy_position.y,
            math.radians(0),
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
            0.0,
        )
        current_states.append(module_state)
    trajectory.set_current_state(current_states)

    desired_states: List[DriveModuleMeasuredValues] = []
    for i in range(len(drive_modules)):
        module_state = DriveModuleDesiredValues(
            drive_modules[i].name,
            math.radians(90),
            1.0,
        )
        desired_states.append(module_state)
    trajectory.set_desired_end_state(desired_states)
    trajectory.align_module_profiles()

    for drive_module in drive_modules:

        state = trajectory.value_for_module_at(drive_module.name, 0.0)
        assert math.isclose(state.drive_acceleration_in_module_coordinates.x, 0.0, rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.drive_velocity_in_module_coordinates.x, 1.0, rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.orientation_velocity_in_body_coordinates.z, math.radians(90), rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.orientation_in_body_coordinates.z, math.radians(0), rel_tol=1e-6, abs_tol=1e-6)

        state = trajectory.value_for_module_at(drive_module.name, 0.5)
        assert math.isclose(state.drive_acceleration_in_module_coordinates.x, 0.0, rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.drive_velocity_in_module_coordinates.x, 1.0, rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.orientation_velocity_in_body_coordinates.z, math.radians(90), rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.orientation_in_body_coordinates.z, math.radians(45), rel_tol=1e-6, abs_tol=1e-6)

        state = trajectory.value_for_module_at(drive_module.name, 1.0)
        assert math.isclose(state.drive_acceleration_in_module_coordinates.x, 0.0, rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.drive_velocity_in_module_coordinates.x, 1.0, rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.orientation_velocity_in_body_coordinates.z, math.radians(90), rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.orientation_in_body_coordinates.z, math.radians(90), rel_tol=1e-6, abs_tol=1e-6)

def test_drive_module_trajectory_should_create_trajectory_for_sideways_to_forwards_transition():
    drive_modules = create_drive_modules()

    trajectory = DriveModuleStateProfile(drive_modules, 1.0)

    current_states: List[DriveModuleMeasuredValues] = []
    for i in range(len(drive_modules)):
        module_state = DriveModuleMeasuredValues(
            drive_modules[i].name,
            drive_modules[i].steering_axis_xy_position.x,
            drive_modules[i].steering_axis_xy_position.y,
            math.radians(90),
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
            0.0,
        )
        current_states.append(module_state)
    trajectory.set_current_state(current_states)

    desired_states: List[DriveModuleMeasuredValues] = []
    for i in range(len(drive_modules)):
        module_state = DriveModuleDesiredValues(
            drive_modules[i].name,
            math.radians(0),
            1.0,
        )
        desired_states.append(module_state)
    trajectory.set_desired_end_state(desired_states)
    trajectory.align_module_profiles()

    for drive_module in drive_modules:

        state = trajectory.value_for_module_at(drive_module.name, 0.0)
        assert math.isclose(state.drive_acceleration_in_module_coordinates.x, 0.0, rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.drive_velocity_in_module_coordinates.x, 1.0, rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.orientation_velocity_in_body_coordinates.z, -math.radians(90), rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.orientation_in_body_coordinates.z, math.radians(90), rel_tol=1e-6, abs_tol=1e-6)

        state = trajectory.value_for_module_at(drive_module.name, 0.5)
        assert math.isclose(state.drive_acceleration_in_module_coordinates.x, 0.0, rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.drive_velocity_in_module_coordinates.x, 1.0, rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.orientation_velocity_in_body_coordinates.z, -math.radians(90), rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.orientation_in_body_coordinates.z, math.radians(45), rel_tol=1e-6, abs_tol=1e-6)

        state = trajectory.value_for_module_at(drive_module.name, 1.0)
        assert math.isclose(state.drive_acceleration_in_module_coordinates.x, 0.0, rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.drive_velocity_in_module_coordinates.x, 1.0, rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.orientation_velocity_in_body_coordinates.z, -math.radians(90), rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.orientation_in_body_coordinates.z, math.radians(0), rel_tol=1e-6, abs_tol=1e-6)

def test_drive_module_trajectory_should_create_trajectory_for_forwards_to_rotation_transition():
    drive_modules = create_drive_modules()

    trajectory = DriveModuleStateProfile(drive_modules, 1.0)

    current_states: List[DriveModuleMeasuredValues] = []
    for i in range(len(drive_modules)):
        module_state = DriveModuleMeasuredValues(
            drive_modules[i].name,
            drive_modules[i].steering_axis_xy_position.x,
            drive_modules[i].steering_axis_xy_position.y,
            math.radians(0),
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
            0.0,
        )
        current_states.append(module_state)
    trajectory.set_current_state(current_states)

    desired_states: List[DriveModuleMeasuredValues] = []
    for i in range(len(drive_modules)):
        module_state = DriveModuleDesiredValues(
            drive_modules[i].name,
            math.radians(45 + i * 90),
            1.0,
        )
        desired_states.append(module_state)
    trajectory.set_desired_end_state(desired_states)
    trajectory.align_module_profiles()

    for i in range(len(drive_modules)):

        drive_module = drive_modules[i]

        state = trajectory.value_for_module_at(drive_module.name, 0.0)
        assert math.isclose(state.drive_acceleration_in_module_coordinates.x, 0.0, rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.drive_velocity_in_module_coordinates.x, 1.0, rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.orientation_velocity_in_body_coordinates.z, math.radians(45 + i * 90), rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.orientation_in_body_coordinates.z, math.radians(0), rel_tol=1e-6, abs_tol=1e-6)

        state = trajectory.value_for_module_at(drive_module.name, 0.5)
        assert math.isclose(state.drive_acceleration_in_module_coordinates.x, 0.0, rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.drive_velocity_in_module_coordinates.x, 1.0, rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.orientation_velocity_in_body_coordinates.z, math.radians(45 + i * 90), rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.orientation_in_body_coordinates.z, math.radians((45 + i * 90) / 2), rel_tol=1e-6, abs_tol=1e-6)

        state = trajectory.value_for_module_at(drive_module.name, 1.0)
        assert math.isclose(state.drive_acceleration_in_module_coordinates.x, 0.0, rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.drive_velocity_in_module_coordinates.x, 1.0, rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.orientation_velocity_in_body_coordinates.z, math.radians(45 + i * 90), rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.orientation_in_body_coordinates.z, math.radians(45 + i * 90), rel_tol=1e-6, abs_tol=1e-6)

def test_drive_module_trajectory_should_create_trajectory_for_sideways_to_rotation_transition():
    drive_modules = create_drive_modules()

    trajectory = DriveModuleStateProfile(drive_modules, 1.0)

    current_states: List[DriveModuleMeasuredValues] = []
    for i in range(len(drive_modules)):
        module_state = DriveModuleMeasuredValues(
            drive_modules[i].name,
            drive_modules[i].steering_axis_xy_position.x,
            drive_modules[i].steering_axis_xy_position.y,
            math.radians(90),
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
            0.0,
        )
        current_states.append(module_state)
    trajectory.set_current_state(current_states)

    desired_states: List[DriveModuleMeasuredValues] = []
    for i in range(len(drive_modules)):
        module_state = DriveModuleDesiredValues(
            drive_modules[i].name,
            math.radians(45 + i * 90),
            1.0,
        )
        desired_states.append(module_state)
    trajectory.set_desired_end_state(desired_states)
    trajectory.align_module_profiles()

    for i in range(len(drive_modules)):

        drive_module = drive_modules[i]

        state = trajectory.value_for_module_at(drive_module.name, 0.0)
        assert math.isclose(state.drive_acceleration_in_module_coordinates.x, 0.0, rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.drive_velocity_in_module_coordinates.x, 1.0, rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.orientation_velocity_in_body_coordinates.z, math.radians(45 + i * 90) - math.radians(90), rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.orientation_in_body_coordinates.z, math.radians(90), rel_tol=1e-6, abs_tol=1e-6)

        state = trajectory.value_for_module_at(drive_module.name, 0.5)
        assert math.isclose(state.drive_acceleration_in_module_coordinates.x, 0.0, rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.drive_velocity_in_module_coordinates.x, 1.0, rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.orientation_velocity_in_body_coordinates.z, math.radians(45 + i * 90) - math.radians(90), rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.orientation_in_body_coordinates.z, math.radians((45 + i * 90 + 90) / 2), rel_tol=1e-6, abs_tol=1e-6)

        state = trajectory.value_for_module_at(drive_module.name, 1.0)
        assert math.isclose(state.drive_acceleration_in_module_coordinates.x, 0.0, rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.drive_velocity_in_module_coordinates.x, 1.0, rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.orientation_velocity_in_body_coordinates.z, math.radians(45 + i * 90) - math.radians(90), rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(state.orientation_in_body_coordinates.z, math.radians(45 + i * 90), rel_tol=1e-6, abs_tol=1e-6)
