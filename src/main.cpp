#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"

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
  /**
   * twiddle: Flag to enable or disable Twiddle algorithm
   * During experimentation, twiddle is set to true. Once parameter (p) and
   * delta parameter (dp) are finalized, twiddle is set to false.
   */
  bool twiddle = false;
  // std::array<double, 3> p = {0.1, 0.00005, 0.5};  // {Kp, Ki, Kd}
  // std::array<double, 3> dp = {0.011, 0.0001, 0.1};
  // std::array<double, 3> p = {0.111, 0.00015, 0.6};  // Experiment: twiddle_n = 1000
  // std::array<double, 3> dp = {0.01089, 0.000099, 0.11};  // Experiment: twiddle_n = 1000
  // std::array<double, 3> p = {0.12189, 0.000249, 0.6};  // Experiment: twiddle_n = 2000
  // std::array<double, 3> dp = {0.011979, 0.0001089, 0.099};  // Experiment: twiddle_n = 2000
  // std::array<double, 3> p = {0.133869, 0.0001401, 0.699};  // Experiment: twiddle_n = 3000
  // std::array<double, 3> dp = {0.0131769, 0.00011979, 0.1089};  // Experiment: twiddle_n = 3000
  // std::array<double, 3> p = {0.147046, 0.00002031, 0.8079};  // Experiment: twiddle_n = 4000
  // std::array<double, 3> dp = {0.0144946, 0.000131769, 0.11979};  // Experiment: twiddle_n = 4000
  // std::array<double, 3> p = {0.177485, 0.000297025, 0.92769};  // Experiment: twiddle_n = 6000
  // std::array<double, 3> dp = {0.0175385, 0.00015944, 0.131769};  // Experiment: twiddle_n = 6000
  std::array<double, 3> p = {0.214316, 0.000312969, 1.05946};  // Experiment: twiddle_n = 8000
  std::array<double, 3> dp = {0.0212216, 0.000192922, 0.144946};  // Experiment: twiddle_n = 8000
  int idx = 0;  // Parameter (p) and delta parameter (dp) index: [0-2]
  int twiddle_idx = 0;
  double tol = 0.001;  // Tolerance or error below 1.0e-10
  int twiddle_case_i = 0;  // Identify Twiddle case
  bool twiddle_case_1 = true;
  bool twiddle_case_2 = true;
  int n = 0;  // Keep track of Twiddle iteration

  /**
   * twiddle_n: Minimum iterations before accumulating Twiddle error
   * During experimentation, used twiddle_n values from:
   *   [500, 600, 800, 1000, 2000, 3000, 4000, 6000, 8000]
   */
  int twiddle_n = 8000;
  double twiddle_err = 0.0;
  double err = 0.0;
  double best_err = 100000.0;
  std::array<double, 3> best_p = p;

  // Initialize the pid variable.
  if (twiddle == true) {
    pid.Init(p[0], p[1], p[2]);
  } else {
    pid.Init(0.214316, 0.000312969, 1.05946);  // Experiment: twiddle_n = 8000
    // pid.Init(0.177485, 0.000297025, 0.92769);  // Experiment: twiddle_n = 6000
    // pid.Init(0.147046, 0.00002031, 0.8079);  // Experiment: twiddle_n = 4000
    // pid.Init(0.12189, 0.000249, 0.6);  // Experiment: twiddle_n = 3000
    // pid.Init(0.12189, 0.000249, 0.6);  // Experiment: twiddle_n = 2000
    // pid.Init(0.111, 0.00015, 0.6);  // Experiment: twiddle_n = 1000
    // pid.Init(0.06, 0.00031, 1.29);
  }

  h.onMessage([&pid, &p, &dp, &idx, &twiddle_idx, &tol, &twiddle_case_i, &twiddle_case_1, &twiddle_case_2,
               &n, &twiddle_n, &twiddle_err, &err, &best_err, &best_p, &twiddle]
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
          double speed_value;
          double throttle_value = 0.3;  // Same as speed value.
          double max_speed = 20.0;
          json msgJson;

          // Speed p = {0.3, 0.002, 0.0};
          speed_value = 0.3 * (max_speed - speed);

          pid.UpdateError(cte);
          steer_value = pid.TotalError();

          if (twiddle == true) {
            if (n == 0) {
              pid.Init(p[0], p[1], p[2]);
            }

            twiddle_err += cte * cte;  // Accumulating Twiddle error
            n++;

            if (n > twiddle_n) {
              err = twiddle_err / twiddle_n;
              if (twiddle_case_1) {  // Twiddle Case 1
                p[idx] += dp[idx];  // Increase parameter range by delta parameter
                twiddle_case_1 = false;
              } else if (twiddle_case_2) {  // Twiddle Case 2
                if (err < best_err) {
                  best_err = err;
                  best_p = p;
                  dp[idx] *= 1.1;
                  twiddle_case_i++;
                } else {
                  p[idx] -= 2 * dp[idx];  // Decrease parameter range by delta parameter (tighter bounds)
                  twiddle_case_2 = false;
                }
              } else {
                if (err < best_err) {
                  best_err = err;
                  best_p = p;
                  dp[idx] *= 1.1;
                } else {
                  p[idx] += dp[idx];
                  dp[idx] *= 0.9;
                }
                twiddle_case_i++;
              }
              if (twiddle_case_i > 0) {
                idx = (idx + 1) % 3;
                twiddle_case_1 = true;
                twiddle_case_2 = true;
                twiddle_case_i = 0;
              }
              twiddle_err = 0;  // Every twiddle_n observations, restart accumulating Twiddle error
              n = 0;
              twiddle_idx++;

              // DEBUG
              // std::cout << "P = [" << p[0] << ", " << p[1] << ", " << p[2] << "]" << std::endl;
              // std::cout << "DP = [" << dp[0] << ", " << dp[1] << ", " << dp[2] << "]" << std::endl;
              // std::cout << "Best Error " << best_err << std::endl;
              // std::cout << "Best P = [" << best_p[0] << ", " << best_p[1] << ", " << best_p[2] << "]" << std::endl;

              double sum_dp = dp[0] + dp[1] + dp[2];
              if (sum_dp > tol) {  // Reset simulator
                std::string reset_msg = "42[\"reset\",{}]";
                ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
              }
            } else {  // end if twiddle_n
              msgJson["steering_angle"] = steer_value;
              msgJson["throttle"] = speed_value;  // Same as speed value.
              auto msg = "42[\"steer\"," + msgJson.dump() + "]";
              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }  // end else twiddle_n
          } else {  // end if twiddle
            msgJson["steering_angle"] = steer_value;
            msgJson["throttle"] = speed_value;  // Same as speed value.
            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }  // end else twiddle
        }  // end if telemetry
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end if websocket message
  });  // end h.onMessage

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
