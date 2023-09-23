#!/usr/bin/python3

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

from __future__ import annotations

import math
import numpy as np
from numpy.linalg import pinv
from typing import List, Tuple

# local
from .drive_module import DriveModule
from .states import DriveModuleDesiredValues, DriveModuleMeasuredValues, BodyMotion

def normalize_angle(angle_in_radians: float) -> float:
    # reduce the angle to [-2pi, 2pi]
    angle = angle_in_radians % (2 * math.pi)

    # Force the angle to the between 0 and 2pi
    angle = (angle + 2 * math.pi) % (2 * math.pi)

    if angle > math.pi:
        angle -= 2 * math.pi

    return angle

def difference_between_angles(starting_angle_in_radians: float, ending_angle_in_radians: float) -> float:
    normalized_start = normalize_angle(starting_angle_in_radians)
    normalized_end = normalize_angle(ending_angle_in_radians)

    diff_angle = normalized_end - normalized_start

    # make sure we get the smallest angle
    if diff_angle > math.pi:
        diff_angle -= 2 * math.pi
    else:
        if diff_angle < -math.pi:
            diff_angle += 2 * math.pi

    return diff_angle

# Abstract class for control models
class ControlModelBase(object):

    def __init__(self):
        pass

    # Forward kinematics
    def body_motion_from_wheel_module_states(self, states: List[DriveModuleMeasuredValues]) -> BodyMotion:
        return BodyMotion(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    # Returns the proposed wheel states which will achieve the given body motion. The list will contain
    # both a forward, i.e. with the steering angle such that the drive motor turns 'forwards', and a
    # reverse state, i.e. with the steering angle such that the drive motor turns 'backwards'.
    def state_of_wheel_modules_from_body_motion(self, state: BodyMotion) -> List[Tuple[DriveModuleDesiredValues]]:
        return []

class SimpleFourWheelSteeringControlModel(ControlModelBase):

    def __init__(self, drive_modules: List[DriveModule]):
        self.modules = drive_modules

        # The state of the drive modules can be found with the following equation:
        #
        #    V_i = |A| * V
        #
        # where
        #
        #  V = The state vector for the robot body = [v_x, v_y, omega]^T
        #  |A| = The state matrix that translates the body state to the drive module state
        #  V_i = The state vector for the drive modules = [v_1_x, v_1_y, v_2_x, v_2_y, ... , v_n_x, v_n_y]
        #
        # the state matrix is an [2 * n ; 3] matrix
        # [
        #    1.0   0.0   -module_1.y
        #    0.0   1.0   module_1.x
        #    1.0   0.0   -module_2.y
        #    0.0   1.0   module_2.x
        #    1.0   0.0   -module_3.y
        #    0.0   1.0   module_3.x
        #    1.0   0.0   -module_4.y
        #    0.0   1.0   module_4.x
        # ]
        arr = []
        for drive_module in drive_modules:
            x_vel = [1.0, 0.0, -1 * drive_module.steering_axis_xy_position.y]
            y_vel = [0.0, 1.0, 1 * drive_module.steering_axis_xy_position.x]

            arr.append(x_vel)
            arr.append(y_vel)

        self.inverse_kinematics_matrix = np.array(arr)
        self.forward_kinematics_matrix = pinv(self.inverse_kinematics_matrix)

    # Forward kinematics
    def body_motion_from_wheel_module_states(self, states: List[DriveModuleMeasuredValues]) -> BodyMotion:
        # To calculate the body state from the module state we need to invert the state equation. Because the state matrix
        # isn't square we can't use the normal matrix inverse, instead we use the pseudo-inverse. This gets us
        #
        #    |A|_* V_i = V
        #
        #  where
        #
        #  |A|_* = pseudo-inverse of |A|

        # Calculate the v_x and v_y for each module, using the module drive velocity and the steering angle
        drive_state_array: List[float] = []
        for state in states:
            v_x, v_y = state.xy_drive_velocity()
            drive_state_array.append(v_x)
            drive_state_array.append(v_y)

        drive_state_vector = np.array(drive_state_array)
        body_state_vector = np.matmul(self.forward_kinematics_matrix, drive_state_vector)

        return BodyMotion(
            body_state_vector[0],
            body_state_vector[1],
            body_state_vector[2],
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,)

    # Inverse kinematics
    def state_of_wheel_modules_from_body_motion(self, state: BodyMotion) -> List[Tuple[DriveModuleDesiredValues]]:
        # Kinematics
        # Literature:
        # - https://www.chiefdelphi.com/t/paper-4-wheel-independent-drive-independent-steering-swerve/107383/5
        # -
        # For wheel i
        #  - velocity = sqrt( (v_x - omega * y_i)^2 + (v_y + omega * x_i)^2 )
        #  - angle = acos( (v_x - omega * y_i) / (velocity) ) = asin( (v_y + omega * x_i) / (velocity) )
        #
        # Angle: 0 < alpha < Pi
        #  The angle also needs a differentiation if it should go between Pi and 2Pi
        #
        # This assumes that (x_i, y_i) is the coordinate for the steering axis. And that the steering axis is in z-direction.
        # And that the wheel contact point is on that steering axis

        body_state_array: List[float] = [
            state.linear_velocity.x,
            state.linear_velocity.y,
            state.angular_velocity.z
        ]
        body_state_vector = np.array(body_state_array)
        drive_state_vector = np.matmul(self.inverse_kinematics_matrix, body_state_vector)

        # Calculate the drive speeds and the ratio between the required wheel velocity and the
        # maximum velocity that the motor can provide.
        drive_velocities: List[float] = []
        scales: List[float] = []
        for i in range(len(self.modules)):

            v_x = drive_state_vector[2 * i + 0]
            v_y = drive_state_vector[2 * i + 1]
            drive_velocity = math.sqrt(pow(v_x, 2.0) + pow(v_y, 2.0))
            drive_velocities.append(drive_velocity)

            # If the scale factor is less than 1 then we want higher velocity than the motor can provide
            # Using the scale factor this way so we can easily multiply by the scale factor to get the maximum allowed
            # velocity later on.
            if not math.isclose(drive_velocity, 0.0, rel_tol=1e-15, abs_tol=1e-15):
                scale = self.modules[i].drive_motor_maximum_velocity / drive_velocity
                scale = scale if (scale < 1.0) else 1.0
            else:
                scale = 1.0
            scales.append(scale)

        scales.sort(reverse=True)
        normalization_factor = scales[0]

        # Assume that the steering angle is between 0 and 2 * pi
        result: List[Tuple[DriveModuleDesiredValues]] = []
        for i in range(len(self.modules)):
            v_x = drive_state_vector[2 * i + 0]
            v_y = drive_state_vector[2 * i + 1]
            drive_velocity = drive_velocities[i]

            if math.isclose(drive_velocity, 0.0, rel_tol=1e-9, abs_tol=1e-9):
                # If the other wheels are moving then we might be rotating around the current wheel, so then rotate with the
                # same rotational velocity as the body, but negative
                #
                # If other wheels aren't moving then maybe we're at a stop
                #
                # In either case we just keep the position of the wheel where it was
                forward_steering_angle = float('infinity')
            else:
                # Calculate the position of the drive wheel.
                #
                # math.acos returns values between 0 and pi
                cos_angle = math.acos(v_x / drive_velocity)

                # math.asin returns values between -1/2 pi and 1/2 pi
                sin_angle = math.asin(v_y / drive_velocity)

                # The acos value decides if the wheel orientation is between 0 - 90 degrees or 90 - 180 degrees, i.e. top and bottom, but
                # doesn't distinguish between left and right
                # the asin value decides if the wheel orientation is between 90 - 0 degrees or 360 - 270 degrees, i.e. left and right
                if cos_angle <= 0.5 * math.pi:
                    if sin_angle < 0:
                        forward_steering_angle = sin_angle #+ 2 * math.pi
                    else:
                        forward_steering_angle = sin_angle
                else:
                    # cos_angle is larger than 1/2 * pi. In that case if the
                    if sin_angle < 0:
                        # In this case we want to mirror the current angle relative to Pi (or 180 degrees)
                        forward_steering_angle = difference_between_angles(cos_angle, math.pi) + math.pi
                    else:
                        forward_steering_angle = cos_angle

                forward_steering_angle = normalize_angle(forward_steering_angle)

            if not math.isinf(forward_steering_angle):
                reverse_steering_angle = normalize_angle(forward_steering_angle + math.pi)
            else:
                reverse_steering_angle = float("-infinity")

            name = self.modules[i].name
            forward_state = DriveModuleDesiredValues(
                name,
                forward_steering_angle,
                drive_velocity * normalization_factor,
            )

            reverse_state = DriveModuleDesiredValues(
                name,
                reverse_steering_angle,
                -1.0 * drive_velocity * normalization_factor,
            )
            result.append((forward_state, reverse_state))

        return result
