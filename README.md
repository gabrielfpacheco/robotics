# Python Robotics Algorithm

This repository contains an initial version of a path planning algorithm based on a desired trapezoidal velocity profile
considering two given collision-free way-points in a confined space (R3) for an autonomous underwater vehicle (AUV). 
The goal is to evolve this repository with new algorithms and applications for different types of problems and robots.  


# Requirements

- Python 3.7.x 
- numpy
- matplotlib

# How to use

1. Clone this repo:

    > git clone https://github.com/gabrielfpacheco/robotics.git

    > cd robotics/auv/

2. Install the required libraries. You can use environment.yml with conda command.

    > conda env create -f environment.yml

3. Execute the main.py script inside the directory.

    > python main.py

4. The unit tests will be performed and an example will be plotted.


# Results

### 3D animation for displacement from an initial position to a final one:

![plot3d](https://github.com/gabrielfpacheco/robotics/raw/master/results/auv/3d_position.gif)

### Position profiles over time (per axis):
    
![positions_axes](https://github.com/gabrielfpacheco/robotics/raw/master/results/auv/positions.png)

### Velocity profiles over time:
   
#### Per axis:
    
![velocities_axes](https://github.com/gabrielfpacheco/robotics/raw/master/results/auv/velocities.png)

#### Vector norm - Absolute values only:

![velocities_norm](https://github.com/gabrielfpacheco/robotics/raw/master/results/auv/velocity_norm.png)


### Acceleration profiles over time (per axis) and the vector norm:

![accelerations_axes](https://github.com/gabrielfpacheco/robotics/raw/master/results/auv/accelerations.png)

#### Vector norm - Absolute values only:

![accelerations_norm](https://github.com/gabrielfpacheco/robotics/raw/master/results/auv/acceleration_norm.png)