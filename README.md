# Unscented Kalman Filter

<p align="center"><img src="media/ukf_fast.gif" width=700 /></p>
 

The scene above is centered on the green ego car while the target cars to track are blue. The traffic cars accelerate, decelerate, and alter their yaw angle to change lanes. Each of the cars in traffic is tracked by an independent UKF instance.

- The red spheres above the cars represent the (noisy) lidar detection cluster centroid.
- The purple lines show the radar measurements with the velocity magnitude along the detected angle.
- The green spheres show the UKF's estimation of the vehicle's current position, as well as its estimated position several timesteps into the future.

Note that the future projection may swing a bit and take some time to stabilize after a lane change. This is due to the assumptions implicit in the constant turn rate and velocity (CTRV) motion model implemented here.

## Overview of the UKF algorithm
The typical Kalman filter is a Bayesian filter, which works well if the variables are normally distributed and have linear transitions at each time step. The _unscented_ Kalman filter, however, allows for non-linear transitions. It accomplishes this by sampling several points (sigma points) distributed by a spreading factor `lambda` about the mean state estimate.

In the [code comments](src/ukf.cpp), I've marked out the following steps which outline the UKF algorithm:

1. Generate sigma (sampling) points in the augmented state space
2. Predict the motion of each sigma point, according to the CTRV model
3. Predict the next state: both mean and covariance matrices
4. - (Lidar) Predict the measurement mean and covariance; calculate the Kalman gain `K`
   - (Radar) Predict the measurement mean `z_pred` and innovation covariance `S`
5. Update the state, by applying the Kalman gain to the residual (distance between the measurement and prediction)
6. Rinse and repeat!

Among all the SFND projects, I enjoyed this one the most. It was quite challenging!

---
## Build and run the project
```
mkdir build && cd build
cmake ..
make
./ukf_highway
```

**Dependencies**
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
