# Study-RPR-robot
## Definition of the task
This repository contains the study of RPR manipulator which analyzes:
1. direct and inverse kinematics analysis (position, velocity, acceleration);
2. analysis of the singular configurations;
3. inverse dynamic analysis (evaluation of the actuators forces/torques);
4. working space determination according to ISO 9946;
5. design a control structure to follow a trajectory;
6. simulation of the assigned task.

![RPR](readme_images/sist_rif.png?raw=true "The robot")

The assigned task is the following:
1. The manipulator is in point P1. It moves to point P2 with **minimum actuation time**.
2. It moves along a trajectory of linear and circular segments approximating the shape of the letter in the figure, until it
reaches point P3. The letter lies in a plane with inclination with respect to all xyz axes. The segments must be opportunely
connected. For each segment define the maximum velocity and, for the whole trajectory, define suitable acceleration ramps
(**Look-Ahead algorithm**).
3. Return to P1 with **minimum actuation time**.

<p float="left">
  <img src="https://github.com/gianandry/Study-RPR-robot/blob/main/readme_images/Task.png" width="200" />
  <img src="https://github.com/gianandry/Study-RPR-robot/blob/main/readme_images/Trajectory_3d.png" width="400" />
</p>


## Running the project
All the project is developed with Matlab and Simulink, with the help of Simscape.
The main script is contained in `RPR_Simulation.m`, which contains all the executed calculations and shows the result plots.
`Project.slx` contains the complete solution with the controllers developed in Simscape, the solution consists of a decentralized control system with cascaded controllers; while `Project_NoControl.slx` without the controllers.

