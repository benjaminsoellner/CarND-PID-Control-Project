#define _USE_MATH_DEFINES
#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;
using namespace std;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main()
{
  uWS::Hub h;

  // Initialize the controllers.
  PID speed_pid, steer_pid;
  // parameters found by trial-and-error
  speed_pid.Init(.1, 1, .0001);
  steer_pid.Init(0.114638203899845, 1.3948260829918, 0.000055);
  // uncomment to learn parameters twiddle
  // steer_pid.InitTwiddle(0.000936507, 0.0279796, 0, 2.63063e-05, 1500);
  // steer_pid.InitTwiddle(0, 0, 0.00005, 0.0000001, 1500);


  h.onMessage([&steer_pid, &speed_pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
			// j[1] is the data JSON object
			double cte = std::stod(j[1]["cte"].get<std::string>());
			double speed = std::stod(j[1]["speed"].get<std::string>());
			double angle = std::stod(j[1]["steering_angle"].get<std::string>());
			// values to be controlled via PID
			double steer_value, throttle_value;
			// Get PID value for steering
			steer_pid.UpdateError(cte);
			steer_value = steer_pid.GetControl();
			// Speed is between 10 and 30 mph depending on how steep the steering angle is
			double target_speed = 20. * (1. - abs(steer_value)) + 10.;
			// Speed PID is goverened by the "CTE" between target speed and current speed
			speed_pid.UpdateError(speed - target_speed);
			throttle_value = speed_pid.GetControl();
			// Debug message
			//std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
			// Output to simulator
			json msgJson;
			msgJson["steering_angle"] = steer_value;
			msgJson["throttle"] = throttle_value;
			auto msg = "42[\"steer\"," + msgJson.dump() + "]";
			//std::cout << msg << std::endl;
			ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
			// Twiddeling, if configured...
			if (steer_pid.Twiddle()) {
				json twiddle_json;
				twiddle_json["controller"] = "steer_pid";
				twiddle_json["Kp"] = steer_pid.Kp;
				twiddle_json["Kd"] = steer_pid.Kd;	
				twiddle_json["Ki"] = steer_pid.Ki;
				std::cout << "Twiddle: " << twiddle_json.dump() << endl;
			}
			if (speed_pid.Twiddle()) {
				json twiddle_json;
				twiddle_json["controller"] = "speed_pid";
				twiddle_json["Kp"] = speed_pid.Kp;
				twiddle_json["Kd"] = speed_pid.Kd;
				twiddle_json["Ki"] = speed_pid.Ki;
				std::cout << "Twiddle: " << twiddle_json.dump() << endl;
			}
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen("127.0.0.1", port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
