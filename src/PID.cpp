#include "PID.h"

/**
 * Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * Initialize PID coefficients (and errors, if needed)
   */
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
}

void PID::UpdateError(double cte) {
  /**
   * Update PID errors based on cte.
   */
  d_error = cte - p_error;  // diff_CTE
  p_error = cte;  // CTE at time t (now)
  i_error += cte;  // int_CTE (now)
  twiddle_error += cte * cte;
  twiddle_count++;
}

double PID::TotalError() {
  /**
   * Calculate and return the total error:
   *   steer = -tau_p * CTE - tau_i * int_CTE - tau_d * diff_CTE
   */
  return -Kp * p_error - Ki * i_error - Kd * d_error;
}

double PID::TwiddleError() {
  /**
   * Calculate and return the accumulated twiddle error
   */
  return twiddle_error;
