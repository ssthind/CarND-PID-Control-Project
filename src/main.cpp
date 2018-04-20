#include <uWS/uWS.h>
#include <iostream>
#include <fstream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

using namespace std;
// for convenience
using json = nlohmann::json;

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


// Resetting the Simulator
void reset_simulator(uWS::WebSocket<uWS::SERVER>& ws){
  std::string msg("42[\"reset\",{}]");
  ws.send(msg.data(),msg.length(), uWS::OpCode::TEXT);
}

int main()
{
  uWS::Hub h;

  PID pid;
  PID pid_throttle;
  // TODO: Initialize the pid variable.

 // pid.Init(0.1, 0.0005, .4);
    //pid.Init(0.3, 0.00085, 27);
    //pid.Init(0.3, 0.0003, 75);
  pid.Init(0.25, 0.00003, 5);
  pid_throttle.Init(0,0,0);
  std::ofstream logfile;
  int step_cnt =0 ;
  
  
  
  h.onMessage([&pid, &pid_throttle, &logfile, &step_cnt](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          double sim_throttle_value = std::stod(j[1]["throttle"].get<std::string>());
          double steer_value, throttle_value;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          //Steering PID CTE error update
          pid.UpdateError(cte);
          steer_value = pid.TotalError();
          //Throttle PID error update
          pid_throttle.UpdateError(fabs(cte));
          throttle_value = 0.3 - pid_throttle.TotalError();
          
          
          // DEBUG
        //writing in log file
          if (!(logfile.is_open())){
            logfile.open ("logfile1.txt", ios::out | ios::app);
          }
          logfile <<++step_cnt<<","<< pid.cumulative_err <<"," <<cte<< "," << steer_value << "," << speed << "," << throttle_value<< "," << sim_throttle_value<< "\r\n";
          if (logfile.is_open()) logfile.close();
          
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            
      // when vehicle get off road reset simulator
          if (fabs(cte)> 10){
            reset_simulator(ws);  
            step_cnt =0 ;
            pid.Init(0.25, 0.00003, 5);
            pid_throttle.Init(0,0,0);
          }
        }
      }
      else {
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

  h.onConnection([&h, &logfile](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
    logfile.open ("logfile1.txt", ios::out | ios::app);
    logfile<< "step:,"<<"cumulative_err:,"<< "CTE:," <<"Steering:," <<"Speed:," <<"throttle:,"<<"sim_throttle:" << "\r\n";
  });

  h.onDisconnection([&h, &logfile](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
	logfile.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
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
