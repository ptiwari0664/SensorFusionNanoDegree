# SFND_Unscented_Kalman_Filter

In this project, the state of multiple cars on a highway is tracked using an Unscented Kalman Filter (UKF). The scene consists of 3 cars that are tracked and an ego vehicle which is equipped with a LiDAR and a RADAR with noisy measurements. The data obtained from these two sensors are fused to track the cars. The ego vehicle is green in colour while the other cars are blue. The ego vehicle is centered on the center lane and all the measurements are relative to it. The cars which are being tracked are accelerating and also changing their steering rates to change lanes. A constant turn rate and velocity (CTRV) model is used in the prediction step for the Unscented Kalman Filter.

<img src="media/ukf_highway_tracked.gif" width="700" height="400" />

In this project, tracking is done only along the X and Y axis and hence the Z measurement of the LiDAR is ignored. The red sphere on the top of the cars represent the X and Y values as per the LiDAR measurement and the purple lines represent the velocity magnitude as per the RADAR measurement. These measurements along with their measurement noise are considered in the update step of the Unscented Kalman Filter. A root mean square (RMS) analysis is performed to validate the accuracy of the tracking algorithm. The GIF above shows the tracking algorithm in action also with its accuracy which is shown in terms of the RMS values.

`main.cpp` is using `highway.h` to create a straight 3 lane highway environment with 3 traffic cars and the main ego car at the center. The viewer scene is centered around the ego car and the coordinate system is relative to the ego car as well. The ego car is green while the other traffic cars are blue. The traffic cars will be accelerating and altering their steering to change lanes. Each of the traffic car's has it's own UKF object generated for it, and will update each indidual one during every time step.

The red spheres above cars represent the (x,y) lidar detection and the purple lines show the radar measurements with the velocity magnitude along the detected angle. The Z axis is not taken into account for tracking, so you are only tracking along the X/Y axis.

<img src="media/ukf_highway.png" width="700" height="400" />

---

## Other Important Dependencies
* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
 * PCL 1.2

## Basic Build Instructions

1. Clone this repo.
2. cd Project_SFND_Unscented_Kalman_Filter
3. Make a build directory: `mkdir build && cd build`
4. Compile: `cmake .. && make`
5. Run it: `./ukf_highway`

## Code Style

Please stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html) as much as possible.

## Generating Additional Data

This is optional!

If you'd like to generate your own radar and lidar modify the code in `highway.h` to alter the cars. Also check out `tools.cpp` to change how measurements are taken, for instance lidar markers could be the (x,y) center of bounding boxes by scanning the PCD environment and performing clustering. This is similar to what was done in Sensor Fusion Lidar Obstacle Detection.
