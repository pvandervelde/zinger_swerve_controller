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

from abc import ABC, abstractmethod
from typing import List, Tuple

# local
from .control_model import ControlModelBase
from .geometry import Vector3
from .states import DriveModuleDesiredValues, BodyMotion

class InvalidMotionCommandException(Exception):
    pass

class MotionCommand(ABC):

    # The timespan over which the command should be executed
    @abstractmethod
    def time_for_motion(self) -> float:
        pass

    # Determine what the body state would be if the robot would execute the current
    # motion command.
    @abstractmethod
    def to_body_state(self, model: ControlModelBase) -> BodyMotion:
        pass

    # Determine what the state of the drive modules would be if the robot would execute
    # the current motion command.
    @abstractmethod
    def to_drive_module_state(self, model: ControlModelBase) -> Tuple[List[DriveModuleDesiredValues]]:
        pass

# Defines a motion command that specifies a motion from the robot body perspective
class BodyMotionCommand(MotionCommand):

    def __init__(
        self,
        time_span: float,
        linear_x_velocity_in_meters_per_second: float,
        linear_y_velocity_in_meters_per_second: float,
        angular_z_velocity_in_radians_per_second: float
        ):
        self.time_span = time_span
        self.linear_velocity = Vector3(linear_x_velocity_in_meters_per_second, linear_y_velocity_in_meters_per_second, 0.0)
        self.angular_velocity = Vector3(0.0, 0.0, angular_z_velocity_in_radians_per_second)

    # The timespan over which the command should be executed
    def time_for_motion(self) -> float:
        return self.time_span

    # Determine what the body state would be if the robot would execute the current
    # motion command.
    def to_body_state(self, model: ControlModelBase) -> BodyMotion:
        return BodyMotion(
            self.linear_velocity.x,
            self.linear_velocity.y,
            self.angular_velocity.z,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,)

    # Determine what the state of the drive modules would be if the robot would execute
    # the current motion command.
    def to_drive_module_state(self, model: ControlModelBase) -> Tuple[List[DriveModuleDesiredValues]]:
        drive_module_potential_states = model.state_of_wheel_modules_from_body_motion(
            BodyMotion(
                self.linear_velocity.x,
                self.linear_velocity.y,
                self.angular_velocity.z,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            ))
        return (
            [x[0] for x in drive_module_potential_states],
            [x[1] for x in drive_module_potential_states],
        )

# Defines a motion command from the robot drive module perspective
class DriveModuleMotionCommand(MotionCommand):

    def __init__(
        self,
        time_span: float,
        desired_states: List[DriveModuleDesiredValues]):
        self.time_span = time_span
        self.desired_states = desired_states

    # The timespan over which the command should be executed
    def time_for_motion(self) -> float:
        return self.time_span

    # Determine what the body state would be if the robot would execute the current
    # motion command.
    def to_body_state(self, model: ControlModelBase) -> BodyMotion:
        return model.body_motion_from_wheel_module_states(self.desired_states)

    # Determine what the state of the drive modules would be if the robot would execute
    # the current motion command.
    def to_drive_module_state(self, model: ControlModelBase) -> Tuple[List[DriveModuleDesiredValues]]:
        return ( self.desired_states, [] )
