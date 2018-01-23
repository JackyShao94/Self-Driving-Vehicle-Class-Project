# Self-Driving-Vehicle-Class-Project

The class project has two tasks, the first task is let the vehicel race on a pre-defined map as fast as possible.
For this task, we used non-linear programming for the trajectory optimization problem
The MATLAB command 'fmincon' is used as the non-linear programming tool

Task 2 is to let the vehicle race on the same track, but with unknown obstacles on the track
For this task, we used the trajectory generated from the first task as the nominal trajectory, such that we can linearize the dynamics, and only work on the error dynamics
The tricky part is to generate the constraints for the track such that we can use MPC to plan the path
