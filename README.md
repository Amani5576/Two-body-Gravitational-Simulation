# Two-body-Gravitational-Simulation

#### The code simulates the gravitational interaction between two objects in space. This is done by calculating the motion of two objects influenced by their mutual gravitational force.

## Dependencies:
Python 3.x
Libraries: 
- astropy
- matplotlib

## Code Structure:

### Imports:
`astropy.units`, `astropy.constants`: Used for astronomical units and constants.
`matplotlib.pyplot`: For plotting.
`math.pi`: Mathematical constant.

### Constants and Initializations:
Defines constants such as velocity, acceleration, and Earth's density.
Initializes object parameters and properties.

### Class Definition:
Object: Defines an object with properties like name, mass, initial parameters; position, velocity, and acceleration.

### Functions:

`next_position`: Calculates the next position of an object based on gravitational forces.

`get_next_position`: Gets the next coordinates of an object influenced by another object's gravitational pull.

`get_distance_between`: Computes the distance between two objects.

`get_step_size`: Determines the optimal step size for the simulation.

`initiate_simulation`: Initializes the 3D plot for visualization.

`simulate`: Updates and displays the simulation.

`simulation_question`: Prompts user to start the simulation.

## Running the Simulation:
Execute the script and follow the prompts to initiate the simulation.
The simulation plots the trajectories of two objects and tracks their distance over time.

## Notes:
Ensure correct units are used throughout the script.
Adjust parameters like step size and threshold distance for optimal simulation performance.
The file can be run by running `python ./Gravitational_Pull_Simulation.py` on any terminal emulator on a computer with Python 3 installed.
