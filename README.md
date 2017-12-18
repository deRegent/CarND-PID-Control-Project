# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

In this project I implemented PID vehicle control.
PID control is an algorithm which set the steering angle in proportion to the "crosstrack error", or CTE. CTE is the lateral distance between the vehicle and the so-called reference trajectory.

The implementation consists of 3 steps, just like in previous term #2 projects: init step, update step and predict step. 

The second and the third steps are quite simple and they were implemented in PID.cpp. 

The update steps takes CTE and updates internal terms as follows: 

```
void PID::UpdateError(double cte) {
	d_error = cte - p_error;
	p_error = cte;
	i_error += cte;
}
```

There the p_error is the proportional term, the d_error is the differential term, and the i_error is the integral term of CTE.

The prediction step outputs the new steering angle using initialization parametres:

```
double PID::TotalError() {
	return -Kp*p_error - Ki*i_error - Kd*d_error;
}

steer_value = pid.TotalError();
```

The most important steps of the PID controller implementation is the first step. This is the step where the parametres Kp, Ki and Kd are being optimized. 
 

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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 


