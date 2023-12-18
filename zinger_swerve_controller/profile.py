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

# local

class TransientVariableProfile(ABC):

    @abstractmethod
    def first_derivative_at(self, time_fraction: float) -> float:
        pass

    @abstractmethod
    def second_derivative_at(self, time_fraction: float) -> float:
        pass

    @abstractmethod
    def third_derivative_at(self, time_fraction: float) -> float:
        pass

    @abstractmethod
    def value_at(self, time_fraction: float) -> float:
        pass

class SingleVariableLinearProfile(TransientVariableProfile):

    def __init__(self, start: float, end: float):
        self.start = start
        self.end = end

    def first_derivative_at(self, time_fraction: float) -> float:
        return self.end - self.start

    def second_derivative_at(self, time_fraction: float) -> float:
        if time_fraction < 0.0:
            return 0.0

        if time_fraction > 1.0:
            return 0.0

        if math.isclose(0.0, time_fraction, rel_tol=1e-2, abs_tol=1e-2):
            return (self.end - self.start) / 0.01

        if math.isclose(1.0, time_fraction, rel_tol=1e-2, abs_tol=1e-2):
            return -(self.end - self.start) / 0.01

        return 0.0

    def third_derivative_at(self, time_fraction: float) -> float:
        if time_fraction < 0.0:
            return 0.0

        if time_fraction > 1.0:
            return 0.0

        if math.isclose(0.0, time_fraction, rel_tol=1e-2, abs_tol=1e-2):
            return (self.end - self.start) / 0.01 / 0.01

        if math.isclose(1.0, time_fraction, rel_tol=1e-2, abs_tol=1e-2):
            return -(self.end - self.start) / 0.01 / 0.01

        return 0.0

    def value_at(self, time_fraction: float) -> float:
        if time_fraction < 0.0:
            return self.start

        if time_fraction > 1.0:
            return self.end

        return (self.end - self.start) * time_fraction + self.start

# see: https://www.mathworks.com/help/robotics/ug/design-a-trajectory-with-velocity-limits-using-a-trapezoidal-velocity-profile.html
class SingleVariableTrapezoidalProfile(TransientVariableProfile):


    def __init__(self, start: float, end: float):
        self.start = start
        self.end = end

        # For a trapezoidal motion profile the progress in the profile
        # is based on the first derrivative, e.g. if the profile is
        # for position then the progress from one position to another
        # is based on the velocity profile
        #
        # The two extremes are:
        # - Constant velocity over the entire time span
        # - Constant acceleration over half the timespan and constant decleration over
        #   the other half of the timespan
        #
        # In the first case the velocity is (endValue - startValue) / timeSpan
        # In the second case the velocity_max is 2 * ((endValue - startValue) / timeSpan)
        # The actual velocity should be in between these values
        #
        # Initially assume that all phases take 1/3 of the total time
        #
        # Profiles are always defined on a relative time span of 1.0, which makes
        # it easy to alter the timespan.
        #
        # v_min = (end - start) / 1.0
        # v_max = 2 * v_min
        #
        # Assume the profile is 1/3rd acceleration, 1/3 constant velocity and
        # 1/3rd deceleration
        #
        # The total distance is equal to the integral of velocity over time. For
        # a trapezoidal profile this means
        #
        # s = 0.5 * v * t_acc + v * t_const + 0.5 * v * t_dec
        #
        # where:
        # - s = distance
        # - v = maximum velocity in the profile
        # - t_acc = time taken to accelerate
        # - t_const = time taken at constant velocity
        # - t_dec = time taken to decelerate
        #
        # s = v * (0.5 * t_acc + t_const + 0.5 * t_dec)
        #
        # Each segment is 1/3 of the total time
        #
        # s = v * 2/3 * t
        #
        # v = 1.5 * s / t
        self.velocity = 1.5 * (end - start) / 1.0

        self.acceleration_phase_ratio = 1/3
        self.constant_phase_ratio = 1/3
        self.deceleration_phase_ratio = 1/3

    def first_derivative_at(self, time_fraction: float) -> float:
        if time_fraction < 0.0:
            return 0.0

        if time_fraction > 1.0:
            return 0.0

        if time_fraction < self.acceleration_phase_ratio:
            # Accelerating
            starting_velocity = 0.0
            velocity_due_to_acceleration = ((self.velocity - starting_velocity) / self.acceleration_phase_ratio) * time_fraction
            return starting_velocity + velocity_due_to_acceleration

        if time_fraction > (self.acceleration_phase_ratio + self.constant_phase_ratio):
            # deccelerating
            starting_velocity = self.velocity
            ending_velocity = 0.0
            velocity_due_to_acceleration = ((ending_velocity - self.velocity) / self.deceleration_phase_ratio) * (time_fraction - (self.acceleration_phase_ratio + self.constant_phase_ratio))
            return starting_velocity + velocity_due_to_acceleration

        return self.velocity

    def second_derivative_at(self, time_fraction: float) -> float:
        if time_fraction < 0.0:
            return 0.0

        if time_fraction > 1.0:
            return 0.0

        if time_fraction < self.acceleration_phase_ratio:
            # Accelerating
            starting_velocity = 0.0
            return (self.velocity - starting_velocity) / self.acceleration_phase_ratio

        if time_fraction > (self.acceleration_phase_ratio + self.constant_phase_ratio):
            # deccelerating
            ending_velocity = 0.0
            return (ending_velocity - self.velocity) / self.deceleration_phase_ratio

        return 0.0

    def third_derivative_at(self, time_fraction: float) -> float:
        if time_fraction < 0.0:
            return 0.0

        if time_fraction > 1.0:
            return 0.0

        if math.isclose(0.0, time_fraction, rel_tol=1e-2, abs_tol=1e-2):
            starting_velocity = 0.0
            return (((self.velocity - starting_velocity) / self.acceleration_phase_ratio) - 0.0) / 0.01

        if math.isclose(time_fraction, self.acceleration_phase_ratio, rel_tol=1e-2, abs_tol=1e-2):
            starting_velocity = 0.0
            return (0.0 - ((self.velocity - starting_velocity) / self.acceleration_phase_ratio)) / 0.01

        if math.isclose(time_fraction, self.acceleration_phase_ratio + self.constant_phase_ratio, rel_tol=1e-2, abs_tol=1e-2):
            ending_velocity = 0.0
            return (((ending_velocity - self.velocity) / self.deceleration_phase_ratio) - 0.0) / 0.01

        if math.isclose(1.0, time_fraction, rel_tol=1e-2, abs_tol=1e-2):
            ending_velocity = 0.0
            return (0.0 - ((ending_velocity - self.velocity) / self.acceleration_phase_ratio)) / 0.01

        return 0.0

    def value_at(self, time_fraction: float) -> float:
        if time_fraction < 0.0:
            return self.start

        if time_fraction > 1.0:
            return self.end

        if time_fraction < self.acceleration_phase_ratio:
            # Accelerating
            starting_velocity = 0.0
            distance_change_from_velocity = starting_velocity * time_fraction
            distance_change_from_acceleration = 0.5 * ((self.velocity - starting_velocity) / self.acceleration_phase_ratio) * time_fraction * time_fraction
            return self.start + distance_change_from_velocity + distance_change_from_acceleration

        distance_due_to_inital_acceleration = 0.5 * self.velocity * self.acceleration_phase_ratio
        if time_fraction > (self.acceleration_phase_ratio + self.constant_phase_ratio):
            # deccelerating
            distance_due_to_constant_velocity = self.velocity * self.constant_phase_ratio

            deceleration_time = time_fraction - (self.acceleration_phase_ratio + self.constant_phase_ratio)
            ending_velocity = 0.0
            distance_due_to_deceleration = self.velocity * deceleration_time + 0.5 * ((ending_velocity - self.velocity) / self.deceleration_phase_ratio) * deceleration_time * deceleration_time
            return self.start + distance_due_to_inital_acceleration + distance_due_to_constant_velocity + distance_due_to_deceleration

        return self.start + distance_due_to_inital_acceleration + (time_fraction - self.acceleration_phase_ratio) * self.velocity

# S-Curve profile
# --> controlled by the second derivative being linear
class SingleVariableSCurveProfile(TransientVariableProfile):


    def __init__(self, start: float, end: float):
        self.start = start
        self.end = end

        #      t_1     t_2  t_3     t_4  t_5       t_6  t_7
        #  |    *______*
        #  |   /        \
        #  |  /          \
        #  | /            \
        #  |/______________\*_______*____________________________
        #  |                         \                /
        #  |                          \              /
        #  |                           \            /
        #  |                            \*________*/
        #  |
        #
        #
        #
        #
        #
        #
        # For s-curve motion profile the progress in the profile
        # is based on the second and third derrivatives, e.g. if the profile is
        # for position then the progress from one position to another
        # is based on the acceleration and jerk profiles
        #
        # It is assumed that the profile has 7 different sections:
        #
        # 1) Positive jerk, increasing acceleration, increasing velocity
        # 2) zero jerk, constant acceleration, increasing velocity
        # 3) negative jerk, decreasing acceleration, increasing velocity
        # 4) zero jerk, zero acceleration, constant velocity
        # 5) negative jerk, increasingly negative acceleration, reducing velocity
        # 6) zero jerk, constant negative acceleration, reducing velocity
        # 7) positive jerk, decreasing negative acceleration, reducing velocity
        #
        # At the start of state 1) and at the end of state 7) the jerk,
        # acceleration and velocity are zero.
        #
        # For now assume that the profile time sections are:
        #
        # 1) 1/8 of the total time
        # 2) 1/8 of the total time
        # 3) 1/8 of the total time
        # 4) 2/8 of the total time
        # 5) 1/8 of the total time
        # 6) 1/8 of the total time
        # 7) 1/8 of the total time
        #
        # Solving the linear equations for distance based on jerk for each section
        # gives the
        #
        # s = j * 10 / 512 * t
        #
        # j =  (s * 512) / (10 * t)
        self.jerk = 512 / 10 * (end - start) / 1.0

        self.positive_acceleration_phase_ratio = 1/8
        self.constant_acceleration_phase_ratio = 1/8
        self.negative_acceleration_phase_ratio = 1/8
        self.constant_phase_ratio = 1/4

        self.t1 = self.positive_acceleration_phase_ratio
        self.t2 = self.t1 + self.constant_acceleration_phase_ratio
        self.t3 = self.t2 + self.negative_acceleration_phase_ratio
        self.t4 = self.t3 + self.constant_phase_ratio
        self.t5 = self.t4 + self.positive_acceleration_phase_ratio
        self.t6 = self.t5  + self.constant_acceleration_phase_ratio
        self.t7 = 1.0

    def first_derivative_at(self, time_fraction: float) -> float:
        if time_fraction < 0.0:
            return 0.0

        if time_fraction > 1.0:
            return 0.0

        if time_fraction < self.t1:
            return 0.5 * self.jerk * time_fraction * time_fraction

        a1 = self.jerk * self.t1
        v1 = 0.5 * a1 * self.t1
        if time_fraction < self.t2:
            return  v1 + a1 * (time_fraction - self.t1)

        a2 = a1
        v2 = v1 + a1 * (self.t2 - self.t1)
        if time_fraction < self.t3:
            return -0.5 * self.jerk * (time_fraction - self.t2) * (time_fraction - self.t2) + a2 * (time_fraction - self.t2) + v2

        v3 = -0.5 * self.jerk * (self.t3 - self.t2) * (self.t3 - self.t2) + a2 * (self.t3 - self.t2) + v2
        if time_fraction < self.t4:
            return v3

        if time_fraction < self.t5:
            return -0.5 * self.jerk * (time_fraction - self.t4) * (time_fraction - self.t4) + v3

        a5 = -self.jerk * (self.t5 - self.t4)
        v5 = -0.5 * self.jerk * (self.t5 - self.t4) * (self.t5 - self.t4) + v3
        if time_fraction < self.t6:
            return a5 * (time_fraction - self.t5) + v5

        a6 = a5
        v6 = a5 * (self.t6 - self.t5) + v5
        return 0.5 * self.jerk * (time_fraction - self.t6) * (time_fraction - self.t6) + a6 * (time_fraction - self.t6) + v6

    def second_derivative_at(self, time_fraction: float) -> float:
        if time_fraction < 0.0:
            return 0.0

        if time_fraction > 1.0:
            return 0.0

        if time_fraction < self.t1:
            return self.jerk * time_fraction

        if time_fraction < self.t2:
            return self.jerk * self.t1

        if time_fraction < self.t3:
            return -self.jerk * (time_fraction - self.t2) + self.jerk * self.t1

        if time_fraction < self.t4:
            return 0.0

        if time_fraction < self.t5:
            return -self.jerk * (time_fraction - self.t4)

        if time_fraction < self.t6:
            return -self.jerk * (self.t5 - self.t4)

        return -self.jerk * (self.t5 - self.t4) + self.jerk * (time_fraction - self.t6)

    def third_derivative_at(self, time_fraction: float) -> float:
        if time_fraction < 0.0:
            return 0.0

        if time_fraction > 1.0:
            return 0.0

        if time_fraction < self.t1:
            return self.jerk

        if time_fraction < self.t2:
            return 0.0

        if time_fraction < self.t3:
            return -self.jerk

        if time_fraction < self.t4:
            return 0.0

        if time_fraction < self.t5:
            return -self.jerk

        if time_fraction < self.t6:
            return 0.0

        return self.jerk

    def value_at(self, time_fraction: float) -> float:
        if time_fraction < 0.0:
            return self.start

        if time_fraction > 1.0:
            return self.end

        if time_fraction < self.t1:
            return 1/6 * self.jerk * math.pow(time_fraction, 3.0) + self.start

        a1 = self.jerk * self.t1
        v1 = 0.5 * a1 * self.t1
        s1 = 1/6 * self.jerk * math.pow(self.t1, 3.0) + self.start
        if time_fraction < self.t2:
            return  v1 * (time_fraction - self.t1) + 0.5 * a1 * (time_fraction - self.t1) * (time_fraction - self.t1) + s1

        a2 = a1
        v2 = v1 + a1 * (self.t2 - self.t1)
        s2 = v1 * (self.t2 - self.t1) + 0.5 * a1 * (self.t2 - self.t1) * (self.t2 - self.t1) + s1
        if time_fraction < self.t3:
            return -1/6 * self.jerk * math.pow(time_fraction - self.t2, 3.0) + 0.5 * a2 * math.pow(time_fraction - self.t2, 2.0) + v2 * (time_fraction - self.t2) + s2

        v3 = -0.5 * self.jerk * (self.t3 - self.t2) * (self.t3 - self.t2) + a1 * (self.t3 - self.t2) + v2
        s3 = -1/6 * self.jerk * math.pow(self.t3 - self.t2, 3.0) + 0.5 * a2 * math.pow(self.t3 - self.t2, 2.0) + v2 * (self.t3 - self.t2) + s2
        if time_fraction < self.t4:
            return v3 * (time_fraction - self.t3) + s3

        s4 = v3 * (self.t4 - self.t3) + s3
        if time_fraction < self.t5:
            return -1/6 * self.jerk * math.pow(time_fraction - self.t4, 3.0) + v3 * (time_fraction - self.t4) + s4

        a5 = -self.jerk * (self.t5 - self.t4)
        v5 = -0.5 * self.jerk * (self.t5 - self.t4) * (self.t5 - self.t4) + v3
        s5 = -1/6 * self.jerk * math.pow(self.t5 - self.t4, 3.0) + v3 * (self.t5 - self.t4) + s4
        if time_fraction < self.t6:
            return 0.5 * a5 * math.pow(time_fraction - self.t5, 2.0) + v5 * (time_fraction - self.t5) + s5

        a6 = a5
        v6 = a5 * (self.t6 - self.t5) + v5
        s6 = 0.5 * a5 * math.pow(self.t6 - self.t5, 2.0) + v5 * (self.t6 - self.t5) + s5
        return 1/6 * self.jerk * math.pow(time_fraction - self.t6, 3.0) + 0.5 * a6 * math.pow(time_fraction - self.t6, 2.0) + v6 * (time_fraction - self.t6) + s6
