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

# local
from .geometry import Point

class DriveModule(object):

    def __init__(
        self,
        name: str,
        steering_link: str,
        drive_link: str,
        steering_axis_xy_position: Point,
        wheel_radius: float,
        wheel_width: float,
        steering_motor_maximum_velocity: float,
        steering_motor_minimum_acceleration: float,
        steering_motor_maximum_acceleration: float,
        drive_motor_maximum_velocity: float,
        drive_motor_minimum_acceleration: float,
        drive_motor_maximum_acceleration: float):

        self.name = name

        self.steering_link_name = steering_link
        self.driving_link_name = drive_link

        # Assume a vertical steering axis that goes through the center of the wheel (i.e. no steering offset)
        self.steering_axis_xy_position = steering_axis_xy_position
        self.wheel_radius = wheel_radius
        self.wheel_width = wheel_width

        self.steering_motor_maximum_velocity = steering_motor_maximum_velocity

        self.steering_motor_minimum_acceleration = steering_motor_minimum_acceleration
        self.steering_motor_maximum_acceleration = steering_motor_maximum_acceleration

        self.drive_motor_maximum_velocity = drive_motor_maximum_velocity

        self.drive_motor_minimum_acceleration = drive_motor_minimum_acceleration
        self.drive_motor_maximum_acceleration = drive_motor_maximum_acceleration

    # Motors
    # Wheel
    # Sensors

