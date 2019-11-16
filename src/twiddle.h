#ifndef TWIDDLE_H
#define TWIDDLE_H

#include <math.h>
#include <iostream>

/*
 * Twiddle
 */
double p[3] = {0.2, 0.0003, 3.0};  // {Kp, Ki, Kd}
double dp[3] = {0.01, 0.0001, 0.05};
int idx = 0;  // Parameter (p) and delta parameter (dp) index: [0-2]

double tol = 0.00001  // Tolerance or error below 1.0e-10

int twiddle_case = 0;  // Identify Twiddle case
int n = 0;  // Keep track of Twiddle iteration
double best_err = 100000.0;


PID Twiddle(PID pid, double err) {
    if (twiddle_case == 0) {  // Twiddle Case 0: Start
        p[idx] += dp[idx];
        pid.Init(p[0], p[1], p[2]);
        best_err = err;
        twiddle_case = 1;
    } else if (twiddle_case == 1) {
        if (err < best_err) {
            dp[idx] *= 1.1;
            best_err = err;
            idx = (idx + 1) % p.size();
        }
    }

}


#endif  // TWIDDLE_H
