import sys
import numpy as np


class TrapezoidalProfile:
    """ This class provides methods and parameters necessary for imposing a velocity trapezoidal profile to a certain
    object navigating from an initial position to a final one on R3 (x,y,z), solving its kinematic equations and
    providing position for a given instant in time.

    """

    def __init__(self, initial_position, final_position, maximum_velocity, constant_acceleration):
        """ Initializes a trapezoidal profile object an its properties

        :param initial_position: 3-dimensional vector representing initial position in [m]
        :param final_position: 3-dimensional vector representing desired final position in [m]
        :param maximum_velocity: maximum allowed velocity (in module) in the trapezoidal profile in [m/s]
        :param constant_acceleration: constant acceleration (in module) in [m/s^2]
        """

        self.__maximum_velocity = maximum_velocity
        self.__constant_acceleration = constant_acceleration
        self.__initial_position = initial_position
        self.__final_position = final_position
        self.__displacement = np.linalg.norm(self.__final_position - self.__initial_position)

        # Minimum duration (in [s]) and displacement (in [m]) necessary to the complete the whole trapezoidal profile.
        # They are calculated considering that there is no constant velocity phase, only acceleration followed by
        # deceleration (triangular profile)
        self.__minimum_duration = 2 * self.__maximum_velocity / self.__constant_acceleration
        self.__minimum_displacement = self.__minimum_duration * self.__maximum_velocity / 2

        self.__set_time_instants()

    def __set_time_instants(self):
        """ Sets the time instants for each phase of the desired profile considering the degenerative case when there is
        no constant velocity phase.

        """

        # Treating degenerative case and adjusting trajectory's minimum displacement and duration
        if self.__displacement <= self.__minimum_displacement:
            self.__minimum_displacement = self.__displacement
            self.__maximum_velocity = np.sqrt(self.__constant_acceleration * self.__minimum_displacement)
            self.__minimum_duration = 2 * self.__maximum_velocity / self.__constant_acceleration

        # Time in [s] at the beginning of acceleration phase (initial time)
        self.__start_time = 0

        # Time in [s] at the end of acceleration phase (beginning of the constant velocity phase)
        self.__end_acceleration_time = self.__start_time + self.__minimum_duration / 2

        # Time in [s] at the end of constant velocity phase (beginning of deceleration phase)
        self.__end_const_velocity_time = self.__end_acceleration_time + \
                                         (self.__displacement - self.__minimum_displacement) / self.__maximum_velocity

        # Time in [s] at the end of deceleration phase (final time)
        self.__end_time = self.__end_const_velocity_time + self.__minimum_duration / 2

    def solve_position_equation(self, time):
        """ Solves the kinematic equations for each phase of the trapezoidal profile, considering the
        constraints in bot velocity and acceleration that a trapezoidal profile imposes.

        :param time: time instant in [s] for which the objects position should be calculated
        :return: 3-dimensional in [m] vector representing the object's position at the given time
        """

        # Unitary vector along the direction of the movement
        unitary_vector = (self.__final_position - self.__initial_position) / self.__displacement

        # Acceleration and velocity constraints expressed in vectors along the direction of the movement
        acceleration = self.__constant_acceleration * unitary_vector
        maximum_velocity = self.__maximum_velocity * unitary_vector

        # Kinematics during acceleration phase
        if self.__start_time <= time <= self.__end_acceleration_time:
            position = self.__initial_position + .5 * acceleration * (time - self.__start_time) ** 2

        # Kinematics during constant velocity phase
        elif self.__end_acceleration_time < time <= self.__end_const_velocity_time:
            position = self.__initial_position + maximum_velocity * \
                       (time - self.__end_acceleration_time / 2 - self.__start_time / 2)

        # Kinematics during deceleration phase
        elif self.__end_const_velocity_time < time <= self.__end_time:
            position = self.__initial_position + \
                       maximum_velocity * (time - self.__end_acceleration_time / 2 - self.__start_time / 2) \
                       - .5 * acceleration * (time - self.__end_const_velocity_time) ** 2
        else:
            error_msg = "The time instant to evaluate AUV's 3D position must to be between {}s and {}s"
            sys.exit(error_msg.format(str(self.__start_time), str(self.__end_time)))

        return position.transpose()

    @property
    def maximum_velocity(self):
        return self.__maximum_velocity

    @property
    def constant_acceleration(self):
        return self.__constant_acceleration

    @property
    def initial_position(self):
        return self.__initial_position

    @property
    def final_position(self):
        return self.__final_position

    @property
    def start_time(self):
        return self.__start_time

    @property
    def end_time(self):
        return self.__end_time

