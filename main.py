import numpy as np
from auv.autonomous_underwater_vehicle import AutonomousUnderwaterVehicle


def main():

    # Creating an instance of an AUV
    auv = AutonomousUnderwaterVehicle(6, 3)

    # Initial and final positions - these would be provided by the library that generates collision-free way-points
    initial_point = np.array([1, -5, 3])[:, np.newaxis]
    final_point = np.array([6, 8, 12])[:, np.newaxis]

    # In order to test the triangular profile one could use the following destination point:
    # final_point = np.array([2, 0, 4])[:, np.newaxis]

    # Generating splines for each axis over time
    trajectory, time_vector = auv.generate_splines_from_trapezoidal_profile(initial_point, final_point)

    # Plotting results
    auv.plot_3d_trajectory(trajectory, time_vector)
    auv.plot_position_profiles(trajectory, time_vector)
    auv.plot_velocity_profiles(trajectory, time_vector)
    auv.plot_acceleration_profiles(trajectory, time_vector)

    print("The program has ended successfully")
#####


if __name__ == '__main__':
    main()
#####
