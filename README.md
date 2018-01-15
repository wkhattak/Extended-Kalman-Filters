# Project: Extended Kalman Filter Project
## Overview  

This project is about utilizing a [Kalman Filter](https://en.wikipedia.org/wiki/Kalman_filter) to estimate the state (position & velocity) of a moving object of interest with noisy LiDAR and Radar measurements. 

## How Does It Work?

This project involves the use of a simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases). The simulator provides LiDAR and Radar measurements that are utilized by the Extended Kalman Filter(EKF)to provide estimated position & velocity of the object of interest.
The EKF works in two steps; *predict* and *update*. In the *predict* step, based on the time difference alone (between previous & current timestamps), a prediction is made, whereas in the *update* step, the belief in object's location is updated based on the received sensor measurements.

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. 

**INPUT**: values provided by the simulator to the c++ program

`sensor_measurement` => the measurement that the simulator observed (either LiDAR or Radar)


**OUTPUT**: values provided by the c++ program to the simulator

`estimate_x` <= kalman filter estimated position x
`estimate_y` <= kalman filter estimated position y
`rmse_x` <= root mean square error of position x
`rmse_y` <= root mean square error of position y
`rmse_vx` <= root mean square error of velocity x
`rmse_vy` <= root mean square error of velocity y

---

## Requirements

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

