#define _USE_MATH_DEFINES
#include "PID.h"
#include <math.h> 
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Kd, double Ki) {
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;
	d_error = 0.0;
	p_error = 0.0;
	i_error = 0.0;
	cte_sum = 0.0;
	ran_for_steps = 0;
	tw_best_avg_cte = -1.0; // i.e., undefined
	ignore_initial_steps = 10;
}

void PID::InitTwiddle(double twiddle_p, double twiddle_d, double twiddle_i, double twiddle_tolerance, unsigned long long twiddle_after) {
	// switch twiddeling on
	twiddeling = true;
	// store tolerance after which twiddeling is stopped and store interval after which parameters are twiddled
	this->twiddle_tolerance = twiddle_tolerance;
	this->twiddle_after = twiddle_after;
	// initial twiddle deltas
	tw_param_diffs[0] = twiddle_p;
	tw_param_diffs[1] = twiddle_d;
	tw_param_diffs[2] = twiddle_i;
	// best average CTE (this is what twiddle optimizes for) 
	tw_best_avg_cte = -1;
}

void PID::UpdateError(double cte) {
	// differential error = current error - previous error
	d_error = cte - p_error;
	// proportional error = now the current error 
	p_error = cte;
	// integral error = add current term
	i_error += cte;
	// collect average error for twiddeling but wait for initial amount of steps
	if (ran_for_steps > ignore_initial_steps)
		cte_sum += pow(cte, 2);
	ran_for_steps++;
}

double PID::TotalError() {
	return cte_sum / (ran_for_steps - ignore_initial_steps);
}

void PID::ResetTotalError() {
	ran_for_steps = 0;
	cte_sum = 0;
}

double PID::GetControl() {
	double control_value = -Kp*p_error - Kd*d_error - Ki*i_error;
	if (control_value < -1) {
		control_value = -1;
	}
	else if (control_value > 1) {
		control_value = 1;
	}
	return control_value;
}

bool PID::Twiddle() {
	// check if twiddle is running and if we just hit a step after which we have to twiddle
	if (twiddeling && (ran_for_steps > twiddle_after)) {
		// get the current total error
		double curr_avg_cte = TotalError();
		// holds decision if after this adjustment we switch to the next twiddleable parameter 
		// (happens only after we tried increasing & decreasing the current parameter at least
		// once)
		bool go_to_next = false;
		// checks whether we just started twiddeling
		if (tw_best_avg_cte < 0) {
			// we just started, so the current CTE is the best guess
			tw_best_avg_cte = curr_avg_cte;
			// make sure we are starting to twiddle the first parameter only
			tw_curr_direction = 0;
			tw_curr_param = 0;
			while (tw_param_diffs[tw_curr_param] == 0) {
				tw_curr_param = (tw_curr_param + 1) % 3;
			} 
		}
		// check whether there is still a significant change in the twiddleable parameters
		if (tw_param_diffs[0] + tw_param_diffs[1] + tw_param_diffs[2] > twiddle_tolerance) {
			// stores the amount the current twiddleable parameter is changed
			double increment_param = 0.0;
			switch (tw_curr_direction) {
			case 0:
				// in case we just started adjusting the current parameter, 
				// just use the current change rate as increment
				increment_param = tw_param_diffs[tw_curr_param];
				// save state noting that we are increasing the parameter
				tw_curr_direction = 1;
				break;
			case 1:
				// we have been increasing recently, so if increasing helps...
				if (curr_avg_cte < tw_best_avg_cte) {
					// store that as the new best CTE
					tw_best_avg_cte = curr_avg_cte;
					// and increase even more next time we increase this parameter
					tw_param_diffs[tw_curr_param] *= 1.1;
					// go to the next parameter
					go_to_next = true;
				}
				else {
					// otherwise, try running twiddle with the algorithm 
					// running in the opposite direction
					increment_param = -2 * tw_param_diffs[tw_curr_param];
					// save state noting that we switched directions
					tw_curr_direction = -1;
				}
				break;
			case -1:
				// we were recently decreasing (and before that increasing), so if that helped...
				if (curr_avg_cte < tw_best_avg_cte) {
					// store that as the new best CTE
					tw_best_avg_cte = curr_avg_cte;
					// go even more into that direction
					tw_param_diffs[tw_curr_param] *= 1.1;
				}
				else {
					// if it did not help, reverse the change and ...
					increment_param = tw_param_diffs[tw_curr_param];
					// ... try some lower change instead
					tw_param_diffs[tw_curr_param] *= 0.9;
				}
				// go to the next parameter
				go_to_next = true;
				break;
			}
			// depending on what parameter we are currently tweaking, change it accordingly
			switch (tw_curr_param) {
			case 0:
				Kp += increment_param;
				break;
			case 1:
				Kd += increment_param;
				break;
			case 2:
				Ki += increment_param;
				break;
			}
			// if we are switching parameters (happens after we tried increasing and, if that doesn't help, decreasing)
			if (go_to_next) {
				// reset the twiddleing direction
				tw_curr_direction = 0;
				// find the next parameter that is not zero (they should not all be zero because 
				// of the if-statement above) 
				do {
					tw_curr_param = (tw_curr_param + 1) % 3;
				} while (tw_param_diffs[tw_curr_param] == 0);
			}
			// as a last step, reset the counter that 
			ResetTotalError();
			// return true = yes, we have been twiddeling
			return true;
		}
		else {
			// return false = no, we have not been twiddeling (because the tolerance level is reached)
			return false;
		}
	}
	else {
		// return false = no, we have not been twiddeling (because not configured or not a twiddle-step)
		return false;
	}
}

