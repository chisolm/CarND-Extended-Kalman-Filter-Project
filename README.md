# Extended Kalman Filter Project Starter Code
Self-Driving Car Engineer Nanodegree Program

In this project you will utilize a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower that the tolerance outlined in the project rubric. 

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF

INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)

OUTPUT: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]

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

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF `

## Code Style

Code has been check to be nearly compliant to Google's C++ Style. [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Rubric points

### Compilation

Code compiles without errors.

Note: it does produce a warning on a missing link directory, I believe this is
an artifact of my setup.

'''
Scanning dependencies of target ExtendedKF
[ 20%] Building CXX object CMakeFiles/ExtendedKF.dir/src/main.cpp.o
[ 40%] Building CXX object CMakeFiles/ExtendedKF.dir/src/tools.cpp.o
[ 60%] Building CXX object CMakeFiles/ExtendedKF.dir/src/FusionEKF.cpp.o
[ 80%] Building CXX object CMakeFiles/ExtendedKF.dir/src/kalman_filter.cpp.o
[100%] Linking CXX executable ExtendedKF
ld: warning: directory not found for option '-L/usr/local/Cellar/libuv/1.11.0/lib'
[100%] Built target ExtendedKF
'''

### Accuracy

Accuracy is within limits for both data sets.

RMSE DataSet 1:
'''
X: 0.0973
Y: 0.0855
VX: 0.4513
VY: 0.4399
'''

RMSE DataSet 2:
'''
X: 0.0726
Y: 0.0967
VX: 0.4579
VY: 0.4966
'''

### Sensor Fusion Algorithm

My sensor fusion algorythm is taken nearly character for character from the class lessons.

### First Measurements

It handles the first measurement and uses it to set position.

I do not set velocity based on a first radar measurement.  I do not think 
there is any guarantee that the velocity vector is in line with the position
vector/angle and therefor there is not enough information to set velocity.

### Kalman Filter Predict then Update

It follows the given order.

### Radar and Lidar measurements

It uses both types of measurements.

### Code efficiancy

I made minimal changes to reduce computation where a value could be computed
statically.

I do have a couple control flow checks that could be removed, but I have left in for debug.  One is the check on dt, indicating a second run on the same 
data.

Some of the remaining component values for the Q matrix could be pre_computed.
However all that is left is 2*noiseaX/Y and those should get picked up by the
compiler.

### Comments on running with single sets of data.

Naturally the accuracy of only running with a single set of data provides
less accuracy.

When run with only Radar data the position accuracy was more the 2x worse
than combined however the velocity error was closer.  Only 20% or so out of the limits for the exercise.

When run with only the Laser data, the position data was quite close to the
RMSE goal, approximately 30% greater.  The velocity RMSE was barely out of goal on VY and 20% out of goal for VX.

Those numbers were taken from dataset 1.

