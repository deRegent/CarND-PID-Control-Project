# CarND-Controls-PID
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

In this project I implemented PID vehicle control.
PID control is an algorithm which set the steering angle in proportion to the "crosstrack error", or CTE. CTE is the lateral distance between the vehicle and the so-called reference trajectory.

## The implementation

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

The most important steps of the PID controller implementation is the first step. This is the step where the parametres Kp, Ki and Kd are being set and optimized. 

The parameter Kp, which is "P" in the PID, helps steer in proportion to the CTE. 
The parameter Ki, which is "I" in the PID, counters the systematic bias from misaligned wheels.
The parameter Kd, which is "D" in the PID, reduces overshooting of the car when CTE were reduced. 

## PID optimization

The final parameters were chosen by manual testing, recording of the track and comparing the results. The process of manual testing was inspired by Twiddle secting in PID lesson of CarND program.
First of all, I set my initial guess as [1 0 0]. I recorded the video with this parameters in CarND term2 simulator. When I multiplied the first parameter by 10 and checked if results are better than the previous. Provided it was not any better, I divided my initial guess by 10, so it become [0.1 0 0]. The result trajectory became better, so I divided by 10 again. The result parametres [0.01 0 0] performed not as well as the previous guess, so I set the P parameter to the 0.1. 

When I did the following procedure to the I and D parametres. The parameters which I received were:
```
P: 0.1
I: 0.000001
D: 10
```
 
The I value were so small that I decided that the error from misaligned wheels was not a big concern in the simulator, so I even tried to set I to 0 and got the same result as it was with 0.000001.

However, I was wondering how the car would do if I increase the throttle value. The default throttle was 0.3, so I tried 0.6, 0.8 and even 1.0. It corresponded to the maximum speed of 60 mph, 80 mph and 100 mph respectively. 
The car did well on the 60 mph, but it failed on 80 and 100. What I discovered from this experiments is that D value in PID tends to overshoot if I set it to the large value of 10, so I took half of it, so the car could go up to 80 mph now. 

My final PID parametres are:
 
```
P: 0.1
I: 0.000001
D: 5
```

[This video](./videos_out/final_result.mp4) shows the result trajectory on the maximum 30 mph speed.

[This video](./videos_out/final_result_0.6.mp4) shows the result trajectory on the maximum 60 mph speed.

[This video](./videos_out/final_result_0.8.mp4) shows the result trajectory on the maximum 80 mph speed.
 

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


