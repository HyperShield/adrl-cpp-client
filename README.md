# AirSim Drone Racing Lab C++ client

The ADRL is meant to be used with python, but with a bit of trickery you can get it running with C++. You will need to have airlib in a folder at the same level as the project folder.

This repository is my attempt to get a sub 30 second time using a bad approach. I'm using a tuned quintic hermite spline as a trajectory that is parameterized based on path length. Then a reactive attitude controller is used to track the trajectory.

I'm also controlling the quadrotor directly with pwm signals instead of relying on the built-in simpleflight autopilot.

# Demonstation

https://user-images.githubusercontent.com/64291228/163844507-f2a63303-2dfa-4cbf-ac2f-76d8ad063a2f.mp4
