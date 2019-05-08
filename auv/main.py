import numpy as np
from auv import AutonomousUnderwaterVehicle


def main():

    # Creating an instance of an AUV
    auv = AutonomousUnderwaterVehicle(6, 3)

    # Initial and final positions - these would be provided by the library that generates collision-free way-points
    p0 = np.array([1, -5, -3])[:, np.newaxis]
    p1 = np.array([6, 3, 10])[:, np.newaxis]

    p_vec, t_vec = auv.generate_splines_from_trapezoidal_profile(p0, p1)

    auv.plot_3d_trajectory(p_vec, t_vec)
    auv.plot_position_profiles(p_vec, t_vec)
    auv.plot_velocity_profiles(p_vec, t_vec)
    auv.plot_acceleration_profiles(p_vec, t_vec)

    # Test 2
    p2 = np.array([8, 5, 13])[:, np.newaxis]

    # p_vec, t_vec = auv.generate_splines_from_trapezoidal_profile(p1, p2, 25)

    # auv.plot_3d_trajectory(p_vec, t_vec)
    # auv.plot_position_profiles(p_vec, t_vec)
    # auv.plot_velocity_profiles(p_vec, t_vec)
    # auv.plot_acceleration_profiles(p_vec, t_vec)

    print("End of program")
#####


if __name__ == '__main__':
    main()
#####
