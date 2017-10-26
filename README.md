# Udacity Project 6: Extended Kalman Filter
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

The goal of this project is to use kalman filter equations to track a bicycle using RADAR and LIDAR measurements.
Because of the characteristics of RADAR and LIDAR sensor data I had to use standard Kalman filter equations as well as Extended Kalman Filter equations.

Project video: (Youtube link)

[![Project track](https://github.com/stefancyliax/CarND-Extended-Kalman-Filter-Project/raw/master/output_images/project_video.gif)](https://www.youtube.com/watch?v=ONBwEsQtf_w)

### Basic principle

The basic working principle of a Kalman filter is to track the state of an object using state and the uncertainty of this state. Then, at some point, a new sensor measurement arrives, again with a certain uncertainty.
To update our state with the new measurement, we first do a prediction of what the state should be like at the given point in time. Then we weight the prediction and the sensor measurement by their distinct uncertaincy. If the measurement is very uncertain, the predicted state is weighted higher and vise versa.
This allows for an algorithm that combines serveral noisy measurements and arrive gradually at an accurate state.

### Project Approach

The starter code was extensive and set the basic structure of the programm. This was very welcome since this is the first C++ project in the SDC Nanodegree. Additionally the [simulator](https://github.com/udacity/self-driving-car-sim/releases) made a comeback for term 2.

Most of the code needed to finish the project was provided in the lessons, so that the project was no big challenge.

### Basic Build Instructions

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF

For dependencies and setup see the Udacity starter [documentation](https://github.com/stefancyliax/CarND-Extended-Kalman-Filter-Project/blob/master/Starter_code_README.md). 