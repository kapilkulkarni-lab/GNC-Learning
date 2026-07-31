Lab 2 is a decently in depth 3 DOF simulation of a USV

In order to properly simulate a USV many different functions had to be written. The mass matrix function considers the ridgid body mass and inertia along with the added mass and inertia from the fluid around the body. 

The coriolis matrix function builds a matrix to account for the coriolis effects caused by the rotating frames of the USV

The dmaping matrix function builds the damping matrix which takes the damping constants for linear and quadratic damping and then considers that with the current body frame velocities, u, v, r

The nu_dot function conducts the dynamics calculations for the USV to find the rate of change of the nu vector, which contrains the u, v, r values. These are all body frame values

The current velocity function provides a constant velocity value which exist only in the surge and sway directions

The eta dot function is finding the world frame velocities using the current body frame velocities utilizing the rotation matrix formula. 

These are the primary calculation functions, the remaining functions in the script are utilized for structure considerations and basic calculations like state_dot and the rk4 integrator. 

The primary for loop steps through each dt and performs the state estimation calculations utilizng the rk4 step and the state dot function and logs these values for each timestep for plotting. 