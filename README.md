# Study-RPR-robot
## Definition of the task
This repository contains the study of RPR manipulator which analyzes:
1. direct and inverse kinematics analysis (position, velocity, acceleration);
2. analysis of the singular configurations;
3. inverse dynamic analysis (evaluation of the actuators forces/torques);
4. working space determination according to ISO 9946;
5. design a control structure to follow a trajectory;
6. simulation of the assigned task.


The assigned task is the following:
1. The manipulator is in point P1.
2. It moves to point P2 with **minimum actuation time**.
3. It moves along a trajectory of linear and circular segments approximating the shape of the letter in the figure, until it
reaches point P3. The letter lies in a plane with inclination with respect to all xyz axes. The segments must be opportunely
connected. For each segment define the maximum velocity and, for the whole trajectory, define suitable acceleration ramps
(**Look-Ahead algorithm**).
4. Return to P1 with **minimum actuation time**.

## Running the project
All the project is developed with Matlab and Simulink, with the help of Simscape.
The main script is contained in `RPR_Simulation.m`, which contains
