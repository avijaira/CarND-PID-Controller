#ifndef TWIDDLE_H
#define TWIDDLE_H

#include <math.h>
#include <iostream>

/*
 * Twiddle
 */
double p[3] = {0.2, 0.0003, 3.0};  // {Kp, Ki, Kd}
double dp[3] = {0.01, 0.0001, 0.05};
int n = 0;
double tol = 0.00001  // Tolerance or error below 1.0e-10


PID Twiddle(PID pid, double err) {

}


#endif  // TWIDDLE_H
