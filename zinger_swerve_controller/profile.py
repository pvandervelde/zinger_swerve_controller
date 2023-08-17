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

# 4th and 5th order s-curve
