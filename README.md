# Two-body-Gravitational-Simulation

## The code simulates the gravitational interaction between two objects in space. This is done by calculating the motion of two objects influenced by their mutual gravitational force.

## Dependencies:
Python 3.x
Libraries: astropy, matplotlib?

## Code Structure:

### a. Imports:
`astropy.units`, `astropy.constants`: Used for astronomical units and constants.
`matplotlib.pyplot`: For plotting.
`math.pi`: Mathematical constant.

### b. Constants and Initializations:
Defines constants such as velocity, acceleration, and Earth's density.
Initializes object parameters and properties.

### c. Class Definition:
Object: Defines an object with properties like name, mass, initial parametres; position, velocity, and acceleration.

### d. Functions:
`next_position`: Calculates the next position of an object based on gravitational forces.
`get_next_position`: Gets the next coordinates of an object influenced by another object's gravitational pull.
`get_distance_between`: Computes the distance between two objects.
`get_step_size`: Determines the optimal step size for the simulation.
`initiate_simulation`: Initializes the 3D plot for visualization.
`simulate`: Updates and displays the simulation.
`simulation_question`: Prompts user to start the simulation.

## 5. Running the Simulation:
Execute the script and follow the prompts to initiate the simulation.
The simulation plots trajectories of two objects and tracks their distance over time.

## 6. Notes:
Ensure correct units are used throughout the script.
Adjust parameters like step size and threshold distance for optimal simulation performance.
