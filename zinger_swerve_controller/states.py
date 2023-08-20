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
from typing import Tuple

# local
from .geometry import Orientation, Point, Vector3

class BodyMotion(object):

    def __init__(
        self,
        linear_x_velocity_in_meters_per_second: float,
        linear_y_velocity_in_meters_per_second: float,
        angular_z_velocity_in_radians_per_second: float,
        linear_x_acceleration_in_meters_per_second_quared: float,
        linear_y_acceleration_in_meters_per_second_quared: float,
        angular_z_acceleration_in_radians_per_second_quared: float,
        linear_x_jerk_in_meters_per_second_cubed: float,
        linear_y_jerk_in_meters_per_second_cubed: float,
        angular_z_jerk_in_radians_per_second_cubed: float,):
        self.linear_velocity = Vector3(linear_x_velocity_in_meters_per_second, linear_y_velocity_in_meters_per_second, 0.0)
        self.angular_velocity = Vector3(0.0, 0.0, angular_z_velocity_in_radians_per_second)
        self.linear_acceleration = Vector3(linear_x_acceleration_in_meters_per_second_quared, linear_y_acceleration_in_meters_per_second_quared, 0.0)
        self.angular_acceleration = Vector3(0.0, 0.0, angular_z_acceleration_in_radians_per_second_quared)
        self.linear_jerk = Vector3(linear_x_jerk_in_meters_per_second_cubed, linear_y_jerk_in_meters_per_second_cubed, 0.0)
        self.angular_jerk = Vector3(0.0, 0.0, angular_z_jerk_in_radians_per_second_cubed)

class BodyState(object):

    # Angles are measured between 0 and 2pi
    def __init__(
        self,
        body_x_in_meters: float,
        body_y_in_meters: float,
        body_orientation_in_radians: float,
        body_linear_x_velocity_in_meters_per_second: float,
        body_linear_y_velocity_in_meters_per_second: float,
        body_angular_z_velocity_in_radians_per_second: float,
        body_linear_x_acceleration_in_meters_per_second_quared: float,
        body_linear_y_acceleration_in_meters_per_second_quared: float,
        body_angular_z_acceleration_in_radians_per_second_quared: float,
        body_linear_x_jerk_in_meters_per_second_cubed: float,
        body_linear_y_jerk_in_meters_per_second_cubed: float,
        body_angular_z_jerk_in_radians_per_second_cubed: float,):

        self.position_in_world_coordinates = Point(body_x_in_meters, body_y_in_meters, 0.0)
        self.orientation_in_world_coordinates = Orientation(0.0, 0.0, body_orientation_in_radians)
        self.motion_in_body_coordinates = BodyMotion(
            body_linear_x_velocity_in_meters_per_second,
            body_linear_y_velocity_in_meters_per_second,
            body_angular_z_velocity_in_radians_per_second,
            body_linear_x_acceleration_in_meters_per_second_quared,
            body_linear_y_acceleration_in_meters_per_second_quared,
            body_angular_z_acceleration_in_radians_per_second_quared,
            body_linear_x_jerk_in_meters_per_second_cubed,
            body_linear_y_jerk_in_meters_per_second_cubed,
            body_angular_z_jerk_in_radians_per_second_cubed)

# Defines the required combination of steering angle and drive velocity for a given drive module in order
# to achieve a given Motion of the robot body.
class DriveModuleDesiredValues(object):

    def __init__(
        self,
        name: str,
        steering_angle_in_radians: float,
        drive_velocity_in_meters_per_second: float,
        ):
        self.name = name
        self.steering_angle_in_radians = steering_angle_in_radians
        self.drive_velocity_in_meters_per_second = drive_velocity_in_meters_per_second

class DriveModuleMeasuredValues(object):

    def __init__(
        self,
        name: str,
        module_x_in_meters: float,
        module_y_in_meters: float,
        steering_angle: float,
        steering_velocity: float,
        steering_acceleration: float,
        steering_jerk: float,
        drive_velocity: float,
        drive_acceleration: float,
        drive_jerk: float
        ):
        self.name = name
        self.position_in_body_coordinates = Point(module_x_in_meters, module_y_in_meters, 0.0)
        self.orientation_in_body_coordinates = Orientation(0.0, 0.0, steering_angle)

        self.drive_velocity_in_module_coordinates = Vector3(drive_velocity, 0.0, 0.0)
        self.orientation_velocity_in_body_coordinates = Vector3(0.0, 0.0, steering_velocity)

        self.drive_acceleration_in_module_coordinates = Vector3(drive_acceleration, 0.0, 0.0)
        self.orientation_acceleration_in_body_coordinates = Vector3(0.0, 0.0, steering_acceleration)

        self.drive_jerk_in_module_coordinates = Vector3(drive_jerk, 0.0, 0.0)
        self.orientation_jerk_in_body_coordinates = Vector3(0.0, 0.0, steering_jerk)

    def xy_drive_velocity(self) -> Tuple[float, float]:
        v_x = self.drive_velocity_in_module_coordinates.x * math.cos(self.orientation_in_body_coordinates.z)
        v_y = self.drive_velocity_in_module_coordinates.x * math.sin(self.orientation_in_body_coordinates.z)

        return v_x, v_y
