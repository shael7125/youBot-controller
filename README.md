# youBot-controller
This repository contains the code I used to build a controller that allowed a simulated youBot to pick and place a block. The controller outputs the configuration at every 0.01 seconds for the entire 30 second trajectory.

My software consists of four files, all contained within "code."
In milestone1, I define NextState, which updates the configuration of the youBot based on
controls and the last configuration. I also define a helper function, getF, which computes
the F matrix for the youBot’s mobile base.
In milestone2, I define TrajectoryGenerator, which outputs a list of transformation matrices
that act as a reference trajectory.
In milestone3, I define FeedbackController, which uses the youBot’s current and reference
end-eRector configuration to produce feedforward + PI controls.
The final file (capstone) integrates these functions. First, capstone generates the reference
trajectory by calling TrajectoryGenerator. Then, the software loops over every timestep. In
the loop, the software computes controls based on the current configuration
(FeedbackController) and then uses these controls to update the configuration.

In "results," three cases are included: best, overshoot, and newTask. "best" is the result of tuning the controller's PI gains to correct initial error as smoothly as possible. "overshoot" corrects the same error, but produces oscillation. Finally, "newTask" generates a new trajectory for the robot to follow and tunes the PI gains similar to the "best" case.

Each case folder includes the .csv file of configuration over the course of the simulated trajectory, as well as a .mov video of the robot following this trajectory. Also included is a .csv file of the end-effector's error from the reference configuration, which is plotted to show the success of my controller. Finally, I include a log file to show how to generate the results shown and a README file that shows which variant of controller I used (feedforward PI, PI, P) as well as the gains I used.

Because I was coding in python, the terminal/log inside VS code isn’t very helpful other
than printing errors. I instead put the sections of code that change depending on the case
(best, overshoot, or New Test) at the top of the capstone.py file. These sections specify Kp,
Ki, initial configuration, and initial and final cube configurations (Tsc0/Tscf).
You can comment/uncomment them based on which case you want to generate .csv files
for.

