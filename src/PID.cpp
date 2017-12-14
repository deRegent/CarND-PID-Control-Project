#include "PID.h"

using namespace std;

/*
* Done: Complete the PID class.
* This class was copied from lesson resources: Self-Driving Car Project Q&A | PID Controller
* https://www.youtube.com/watch?v=YamBuzDjrs8&list=PLAwxTw4SYaPnfR7TzRZN-uxlxGbqxhtm2&index=4
* which was sent to me by my mentor
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;

	p_error = 0;
	i_error = 0;
	d_error = 0;
}

void PID::UpdateError(double cte) {
	d_error = cte - p_error;
	p_error = cte;
	i_error += cte;
}

double PID::TotalError() {
	return -Kp*p_error - Ki*i_error - Kd*d_error;
}

