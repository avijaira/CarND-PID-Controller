#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"
//#include "twiddle.h"

// for convenience
using nlohmann::json;
using std::string;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;

  PID pid;  // Steer PID
  double p[3] = {0.2, 0.0003, 3.0};  // {Kp, Ki, Kd}
  double dp[3] = {0.01, 0.0001, 0.05};
  int idx = 0;  // Parameter (p) and delta parameter (dp) index: [0-2]
  int twiddle_idx = 0;
  double tol = 0.00001;  // Tolerance or error below 1.0e-10
  int twiddle_case_i = 0;  // Identify Twiddle case
  bool twiddle_case_1 = true;
  bool twiddle_case_2 = true;
  int n = 0;  // Keep track of Twiddle iteration
  int twiddle_n = 500;  // Minimum iterations before accumulating Twiddle error
  double twiddle_err = 0.0;
  double err = 0.0;
  double best_err = 100000.0;

  // Initialize the pid variable.
  pid.Init(p[0], p[1], p[2]);

  h.onMessage([&pid, &p, &dp, &idx, &twiddle_idx, &tol, &twiddle_case_i, &twiddle_case_1, &twiddle_case_2,
               &n, &twiddle_n, &twiddle_err, &err, &best_err]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {

    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value;
          double throttle = 0.3;  // Same as speed value.

          /**
           * Calculate steering value here, remember the steering value is [-1, 1].
           * Maybe use another PID controller to control the speed!
           */
          if (n == 0) {
            pid.Init(p[0], p[1], p[2]);
          }

          pid.UpdateError(cte);
          steer_value = pid.TotalError();

          twiddle_err += cte * cte;
          n++;
          if (n > twiddle_n) {
            err = twiddle_err / twiddle_n;
            if (twiddle_case_1) {  // Twiddle Case 1
              p[idx] += dp[idx];  // Increase parameter range by delta parameter
              twiddle_case_1 = false;
            } else if (twiddle_case_2) {  // Twiddle Case 2
              if (err < best_err) {
                best_err = err;
                dp[idx] *= 1.1;
                twiddle_case_i++;
              } else {
                p[idx] -= 2 * dp[idx];  // Decrease parameter range by delta parameter (tighter bounds)
                twiddle_case_2 = false;
              }
            } else {
              if (err < best_err) {
                best_err = err;
                dp[idx] *= 1.1;
              } else {
                p[idx] += dp[idx];
                dp[idx] *= 0.9;
              }
              twiddle_case_i++;
            }
            if (twiddle_case_i > 0) {
              idx = (idx + 1) % p.size();
              twiddle_case_1 = true;
              twiddle_case_2 = true;
              twiddle_case_i = 0;
            }
            twiddle_err = 0;  // Every twiddle_n observations, restart accumulating Twiddle error
            n = 0;
            twiddle_idx++;

            double sum_dp = sum(dp);
            if (sum_dp > tol) {  // Reset simulator
                std::string reset_msg = "42[\"reset\",{}]";
                ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
            }
          }

          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;  // Same as speed value.
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // endif telemetry
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // endif websocket message
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}
