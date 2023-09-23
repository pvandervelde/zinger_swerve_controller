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
import math
from typing import Callable, Mapping, List

from .errors import IncompleteTrajectoryException
from .drive_module import DriveModule
from .profile import TransientVariableProfile
from .states import BodyState, DriveModuleDesiredValues, DriveModuleMeasuredValues, BodyMotion

class BodyMotionProfile(object):

    def __init__(
            self,
            current: BodyState,
            desired: BodyMotion,
            min_trajectory_time_in_seconds: float,
            motion_profile_func: Callable[[float, float], TransientVariableProfile]):
        self.start_state = current
        self.end_state = desired
        self.min_trajectory_time_in_seconds = min_trajectory_time_in_seconds

        self.profile = [
            motion_profile_func(current.motion_in_body_coordinates.linear_velocity.x, desired.linear_velocity.x),
            motion_profile_func(current.motion_in_body_coordinates.linear_velocity.y, desired.linear_velocity.y),
            motion_profile_func(current.motion_in_body_coordinates.linear_velocity.z, desired.linear_velocity.z),

            motion_profile_func(current.motion_in_body_coordinates.angular_velocity.x, desired.angular_velocity.x),
            motion_profile_func(current.motion_in_body_coordinates.angular_velocity.y, desired.angular_velocity.y),
            motion_profile_func(current.motion_in_body_coordinates.angular_velocity.z, desired.angular_velocity.z),
        ]

    def body_motion_at(self, time_fraction: float) -> BodyMotion:
        return BodyMotion(
            self.profile[0].value_at(time_fraction),
            self.profile[1].value_at(time_fraction),
            self.profile[5].value_at(time_fraction),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )

    def time_span(self) -> float:
        return self.min_trajectory_time_in_seconds

class ModuleStateProfile(ABC):

    @abstractmethod
    def align_module_profiles(self):
        pass

    @abstractmethod
    def time_span(self) -> float:
        pass

    @abstractmethod
    def value_for_module_at(self, id: str, time_fraction: float) -> DriveModuleMeasuredValues:
        pass

class DriveModuleStateProfile(ModuleStateProfile):

    def __init__(
            self,
            drive_modules: List[DriveModule],
            min_trajectory_time_in_seconds: float,
            motion_profile_func: Callable[[float, float], TransientVariableProfile]):
        self.modules = drive_modules
        self.motion_profile_func = motion_profile_func
        self.start_states: List[DriveModuleMeasuredValues] = []
        self.end_states: List[DriveModuleDesiredValues] = []
        self.min_trajectory_time_in_seconds = min_trajectory_time_in_seconds

        # Kinda want a constant jerk profile
        self.profiles: Mapping[str, List[TransientVariableProfile]] = {}

    def align_module_profiles(self):
        if len(self.start_states) == 0 or len(self.end_states) == 0:
            raise IncompleteTrajectoryException()

        # for each profile adjust it in time such that none of the velocities / accelerations are too high for the motors to handle
        # Then scale the profiles to match in time.
        pass

    def _create_profiles(self):
        if len(self.start_states) == 0:
            return

        if len(self.end_states) == 0:
            return

        self.profiles.clear()
        for i in range(len(self.modules)):
            start = self.start_states[i]
            end = self.end_states[i]

            end_steering_angle = end.steering_angle_in_radians if not math.isinf(end.steering_angle_in_radians) else start.orientation_in_body_coordinates.z
            module_profiles = [
                # Orientation
                self.motion_profile_func(start.orientation_in_body_coordinates.z, end_steering_angle),

                # Drive velocity
                self.motion_profile_func(start.drive_velocity_in_module_coordinates.x, end.drive_velocity_in_meters_per_second),
            ]

            self.profiles[self.modules[i].name] = module_profiles

    def set_current_state(self, states: List[DriveModuleMeasuredValues]):
        if len(states) != len(self.modules):
            raise ValueError(f"The length of the drive module states list ({ len(states) }) does not match the number of drive modules.")

        self.start_states = states
        self._create_profiles()

    def set_desired_end_state(self, states: List[DriveModuleDesiredValues]):
        if len(states) != len(self.modules):
            raise ValueError(f"The length of the drive module states list ({ len(states) }) does not match the number of drive modules.")

        self.end_states = states
        self._create_profiles()

    def time_span(self) -> float:
        return self.min_trajectory_time_in_seconds

    def value_for_module_at(self, id: str, time_fraction: float) -> DriveModuleMeasuredValues:
        if len(self.start_states) == 0 or len(self.end_states) == 0:
            raise IncompleteTrajectoryException()

        if not id in self.profiles:
            raise ValueError(f"There are no profiles for a drive module with name { id }")

        steering_module: DriveModule = None
        for x in self.modules:
            if x.name == id:
                steering_module = x
                break

        profiles = self.profiles[id]

        return DriveModuleMeasuredValues(
            steering_module.name,
            steering_module.steering_axis_xy_position.x,
            steering_module.steering_axis_xy_position.y,
            profiles[0].value_at(time_fraction),
            profiles[0].first_derivative_at(time_fraction),
            profiles[0].second_derivative_at(time_fraction),
            profiles[0].third_derivative_at(time_fraction),
            profiles[1].value_at(time_fraction),
            profiles[1].first_derivative_at(time_fraction),
            profiles[1].second_derivative_at(time_fraction),
        )
