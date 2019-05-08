import numpy as np
import unittest
from auv.autonomous_underwater_vehicle import AutonomousUnderwaterVehicle
from auv.trapezoidal_profile import TrapezoidalProfile


class TrapezoidalProfileUnitTests(unittest.TestCase):

    def test_degenerative_behavior(self):
        """
        Tests the degenerative case when trapezoidal profile needs to be reduced to a triangular one
        """
        trap_prf = TrapezoidalProfile(1, 10, 6, 3)

        self.assertTrue(trap_prf.end_acceleration_time == trap_prf.end_const_velocity_time)

    def test_solve_position_time_within_bounds(self):
        """
        Tests if there is no error on the position solving equation when time parameters are correct
        """
        TrapezoidalProfile(maximum_velocity=6, constant_acceleration=3).solve_position_equation(2)

    def test_solve_position_time_without_bounds(self):
        """
        Tests if the right treatment is performed if a time in [s] out of profile bounds is passed as an argument
        """
        with self.assertRaises(SystemExit):
            TrapezoidalProfile(maximum_velocity=6, constant_acceleration=3).solve_position_equation(10)


class AuvUnitTests(unittest.TestCase):

    def test_splines_generation(self):
        """
        Tests if there are no errors on the splines generation for the AUV to move through collision-free way-points
        """

        # Creating an instance of an AUV
        auv = AutonomousUnderwaterVehicle(6, 3)

        # Initial and final positions - these would be provided by the library that generates collision-free way-points
        initial_point = np.array([1, -5, -3])[:, np.newaxis]
        final_point = np.array([6, 3, 10])[:, np.newaxis]

        auv.generate_splines_from_trapezoidal_profile(initial_point, final_point)

    def test_3d_plot(self):
        """
        Tests if there are no errors on the plotting
        """

        time_vector = np.linspace(0, 10, 100)
        trajectory = np.ones((3, len(time_vector)))
        AutonomousUnderwaterVehicle(6, 3).plot_3d_trajectory(trajectory, time_vector, False)

    def test_position_profiles_plot(self):
        """
        Tests if there are no errors on the plotting
        """

        time_vector = np.linspace(0, 10, 100)
        trajectory = np.ones((3, len(time_vector)))
        AutonomousUnderwaterVehicle(6, 3).plot_position_profiles(trajectory, time_vector, False)

    def test_velocity_profiles_plot(self):
        """
        Tests if there are no errors on the plotting
        """

        time_vector = np.linspace(0, 10, 100)
        trajectory = np.ones((3, len(time_vector)))
        AutonomousUnderwaterVehicle(6, 3).plot_velocity_profiles(trajectory, time_vector, False)

    def test_acceleration_profiles_plot(self):
        """
        Tests if there are no errors on the plotting
        """

        time_vector = np.linspace(0, 10, 100)
        trajectory = np.ones((3, len(time_vector)))
        AutonomousUnderwaterVehicle(6, 3).plot_acceleration_profiles(trajectory, time_vector, False)


if __name__ == '__main__':
    unittest.main()
