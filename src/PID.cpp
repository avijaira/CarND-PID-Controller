#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
  d_error = cte - p_error;    // diff_CTE
  p_error = cte;    // CTE at time t (now)
  i_error += cte;    // int_CTE (now)

  // total_error += cte * cte;    // NOTE_AV: should we calculate this in TotalError()?
  // cycles++;
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  // return total_error;    // NOTE_AV: maybe it should be calculated here?
  // steer = -tau_p * CTE - tau_i * int_CTE - tau_d * diff_CTE
  return -Kp * p_error - Ki * i_error - Kd * d_error;
}
