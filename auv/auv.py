import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from trapezoidal_profile import TrapezoidalProfile


class AutonomousUnderwaterVehicle:
    """ This class is intended to describe the behavior of an AUV object, containing the methods and properties
    necessary for it to generate a smooth trajectory through given way-points within a confined space (storage tank)
    respecting an imposed velocity profile.

    """

    def __init__(self, auv_max_velocity=8, auv_acceleration=4, tank_height=15, tank_radius=10):
        """ Initializes an AUV object and its properties.

        :param auv_max_velocity: maximum translational velocity (in module) achieved by the AUV in [m/s]
        :param auv_acceleration:  constant acceleration (in module) of the AUV in [m/s^2]
        :param tank_height: storage tank's height in [m]
        :param tank_radius: storage tank's radius in [m]

        """

        self.__auv_max_velocity = auv_max_velocity
        self.__auv_acceleration = auv_acceleration
        self.__tank_height = tank_height
        self.__tank_radius = tank_radius

        # Sampled time vector in which the trajectory 3-dimensional positions are evaluated
        self.__time_vector = None

        # 3-dimensional position vector representing how positions evolves over time
        self.__trajectory = None

    def __check_map_boundaries(self, position):
        """ Checks whether or not a specific point is contained by the map. The storage tank is assumed to be a cylinder
        and the origin of the coordinates system is centered on the its base.

        :param position: 3-dimensional position vector
        :return: boolean returning True if the point is within limits and False if not

        """

        is_within_height = position[-1] <= self.__tank_height
        is_within_circle = np.sqrt(position[0] ** 2 + position[1] ** 2) <= self.__tank_radius

        return is_within_height and is_within_circle

    def generate_splines_from_trapezoidal_profile(self, initial_position, final_position, sampling_factor=10):
        """ Generate splines for the 3 axis from p0 to p1

        :param initial_position: 3-dimensional position vector representing robot's initial position in [m]
        :param final_position: 3-dimensional position vector representing robot's initial position in [m]
        :param sampling_factor: number of desired points per second. Necessary sampling purposes
        :return: position splines in a 3-dimensional vector (one component per axis) and a sampled time vector
        """

        # Check map boundaries for provided initial position
        if not self.__check_map_boundaries(initial_position):
            error_msg = "The origin point {} is not contained within the tank's boundaries"
            sys.exit(error_msg.format(initial_position.T))

        # Check map boundaries for provided final position
        if not self.__check_map_boundaries(final_position):
            error_msg = "The destination point {} is not contained within the tank's boundaries"
            sys.exit(error_msg.format(final_position.T))

        # Creating an instance of the trapezoidal velocity profile
        trapezoidal_profile = TrapezoidalProfile(initial_position, final_position,
                                                 self.__auv_max_velocity, self.__auv_acceleration)
        number_points = int((trapezoidal_profile.end_time - trapezoidal_profile.start_time + 1) * sampling_factor)
        time = np.linspace(trapezoidal_profile.start_time, trapezoidal_profile.end_time, number_points)

        splines = np.zeros((3, len(time)))
        for index, t in enumerate(time ):
            splines[:, index] = trapezoidal_profile.solve_position_equation(t)

        self.__trajectory = splines
        self.__time_vector = time

        return splines, time

    @staticmethod
    def plot_3d_trajectory(trajectory, time):
        """ Plots an animation of how the AUV's position evolves over time. The acceleration, constant velocity and
        deceleration phases can be remarked during the animation.

        :param trajectory: sequence of 3-dimensional position data
        :param time: time vector containing the sampled time interval
        """

        interval = np.diff(time)[0]
        plt.figure()
        ax = plt.axes(projection='3d')
        ax.scatter3D(trajectory[0, 0], trajectory[1, 0], trajectory[2, 0], 'o')
        ax.scatter3D(trajectory[0, -1], trajectory[1, -1], trajectory[2, -1], 'o')

        for i in range(len(time)):
            ax.plot3D([trajectory[0, 0], trajectory[0, i]], [trajectory[1, 0], trajectory[1, i]],
                      [trajectory[2, 0], trajectory[2, i]], '--r')
            plt.draw()
            plt.pause(interval)

        plt.show(block=True)

    @staticmethod
    def plot_position_profiles(trajectory, time):
        """ Plots the position trajectories (splines) over time for each one of the axis.

        :param trajectory: sequence of 3-dimensional position data
        :param time: time vector containing the sampled time interval
        """

        plt.figure()
        plt.plot(time, trajectory[0], ':.')
        plt.plot(time, trajectory[1], ':.')
        plt.plot(time, trajectory[2], ':.')
        plt.grid(True)
        plt.show()

    @staticmethod
    def plot_velocity_profiles(trajectory, time):
        """ Plots the velocity over time for each one of the axis and for their combined velocity (vector norm)

        :param trajectory: sequence of 3-dimensional position data
        :param time: time vector containing the sampled time interval
        """

        vx = calculate_time_derivative(trajectory[0], time)
        vy = calculate_time_derivative(trajectory[1], time)
        vz = calculate_time_derivative(trajectory[2], time)
        v = np.sqrt(vx ** 2 + vy ** 2 + vz ** 2)

        plt.figure()
        plt.plot(time, vx)
        plt.plot(time, vy)
        plt.plot(time, vz)
        plt.grid(True)
        plt.show()

        plt.figure()
        plt.plot(time, v)
        plt.grid(True)
        plt.show()

    @staticmethod
    def plot_acceleration_profiles(trajectory, time):
        """ Plots the acceleration over time for each one of the axis and for their combined acceleration (vector norm)

        :param trajectory: sequence of 3-dimensional position data
        :param time: time vector containing the sampled time interval
        """

        ax = calculate_time_derivative(calculate_time_derivative(trajectory[0], time), time)
        ay = calculate_time_derivative(calculate_time_derivative(trajectory[1], time), time)
        az = calculate_time_derivative(calculate_time_derivative(trajectory[2], time), time)
        a = np.sqrt(ax ** 2 + ay ** 2 + az ** 2)

        plt.figure()
        plt.plot(time, ax)
        plt.plot(time, ay)
        plt.plot(time, az)
        plt.grid(True)
        plt.show()

        plt.figure()
        plt.plot(time, a)
        plt.grid(True)
        plt.show()

    @property
    def v_max(self):
        return self.__auv_max_velocity

    @property
    def a_cte(self):
        return self.__auv_acceleration

    @property
    def trajectory(self):
        return self.__trajectory

    @property
    def time_vector(self):
        return self.__time_vector


def calculate_time_derivative(sig, time):
    """ Auxiliary function that calculates the derivative of a generic signal with respect to a time array

    :param sig: signal array to be differentiated over time
    :param time: time array containing the sampled time interval
    :return: time derivative of sig concatenated of an initial zero to maintain the original number of  points

    """

    derivative = np.diff(sig) / np.diff(time)
    return np.concatenate((np.zeros(1), derivative))
