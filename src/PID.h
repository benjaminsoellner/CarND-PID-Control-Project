#define _USE_MATH_DEFINES
#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  /*
  * Parameters for twiddeling
  */
  bool twiddeling;
  unsigned long long ran_for_steps;
  unsigned long long ignore_initial_steps;
  unsigned long long twiddle_after;
  double twiddle_tolerance;
  double cte_sum;
  double tw_param_diffs[3] = { 0.0, 0.0, 0.0 };
  double tw_best_avg_cte;
  short tw_curr_direction;
  short tw_curr_param;
  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Kd, double Ki);

  /*
  * Initialize PID's twiddeling.
  */
  void InitTwiddle(double twiddle_p, double twiddle_d, double twiddle_i, double twiddle_tolerance, unsigned long long twiddle_after);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*
  * Get Controller Value 
  */
  double GetControl();

  /*
  * Reset total error
  */
  void ResetTotalError();

  /*
  * Twiddles the Kp, Kd, Ki values every "twiddle_after" number of steps. Call this method after each GetControl()
  */
  bool Twiddle();
};

#endif /* PID_H */
