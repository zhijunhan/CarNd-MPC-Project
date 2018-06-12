# Autonomous Driving with Model Predictive Control

## Objective

The goal of this project is to implement Model Predictive Control to navigate a car in a game simulator, which communicates telemetry and track waypoint data thru websocket and sends steering and acceleration commands calculated by MPC back to the simulator. The solution must be robust to 100ms latency, as it might encounter in real-world application.

As learning materials suggest, IPOPT and CPPAD libraries are used to calculate an optimal trajectory and its associated actuation commands in order to minimize cost and error with a third-order polynomial fit to the given waypoints. The optimization is based mostly on the vehicle cross-track error, the orientation angle error and actuation commands related concerns for the purpose of performance improvement and robustness.

## Model and System

- **The Model**

A kinematic vehicle model is implemented to represent the vehicle in the simulator, which basically simplify a dynamic model by ignoring the tire forces, gravity and mass. The kinematic model suffices in this project owing to its tractable, even though there is a compromise of the model accuracy.

  **States:**
  - x: vehicle x coordinate from car's perspective
  - y: vehicle y coordinate from car's perspective
  - ψ(psi): vehicle orientation angle
  - v: vehicle velocity
  - cte: cross track error
  - eψ: orientation error
  
  **Actuations:**
  - δ (delta): steering angle
  - a: acceleration/deceleration, throttle and brake
  
The model combines the states and actuations from the previous timestep to predict the states for the current timestep based on the update equations below:

![equations](./model.png)

## Timestep Length and Elapsed Duration (N & dt)

  * N = 10
  * dt = 0.1

The prediction horizon is the duration over which how far the MPC predicts, T = N * dt. In this case, T equals to 1 second. Beyond a few seconds, the environment will change enough that it won't make sense to predict any further. Timestep length and time elapse are hyperparameters that need to be tuned for the controller. The general guideline would be: elapsed duration shoulbe be as large as possible, while dt should be as small as possible, this brings tradeoffs.

## Polynomial Fitting and MPC Preprocessing:

The reference waypoints are given in global coordinate. In order to simplify the process of polynomial fitting, waypoints need to be converted to vehicle's perspective, as vehicle's x and y positions are at the origin (0, 0) and orientation angle is also zero.

## Model Predictive Control with Latency

Given that a real car won't be executing any actuation command instantly, there is alway delays as command propagates through the system. A delay of 100ms should be fairly realistic, and the 100ms latency is handled in MPC (MPC.cpp lines 120-123).

Additionally, the cost function will punish not just cte, epsi, the difference between velocity and a reference velocity, actuations, as suggested in the lessons, an additional cost (combination of velocity and steer angle) should be taken into consideration during vehicle's passing sharp turning corners.
# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
