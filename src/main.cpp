#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;


int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }
  
  int lane = 1; //starting in center lane
  
  double ref_vel = 0.0;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &lane, &ref_vel]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
                
                
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          int prev_size = previous_path_x.size();
          
          if (prev_size > 0)
          {
            car_s = end_path_s;            
          }
          
          bool car_ahead = false;
          double car_vel = 0.0;
          bool car_left = false;
          bool car_right = false;
          
          for (int i = 0; i < sensor_fusion.size(); i++)
          {
            float d = sensor_fusion[i][6];
            int car_lane = -1;
            
            if (d > 0 && d < 4)
            {
              car_lane = 0;
            }
            else if (d > 4 && d <8)
            {
              car_lane = 1;
            }
            else if (d > 8 && d < 12)
            {
              car_lane = 2;
            }
            if (car_lane < 0)
            {
              continue;
            }
            
            double velx = sensor_fusion[i][3];
            double vely = sensor_fusion[i][4];
            
            double speedcheck = sqrt(velx*velx + vely*vely);
            double scheck = sensor_fusion[i][5];
            //checking which lane and how far car will be in +/- 30 mins
            scheck += ((double) prev_size * 0.02 * speedcheck);
            
            if (car_lane == lane)
            {
              car_ahead = car_ahead | ((scheck > car_s) && (scheck - car_s < 30));    
              if (car_ahead)
              {
                car_vel = speedcheck;
              }
            }
            else if (car_lane - lane == -1) //if car is in the left lane
            {
              car_left = car_left | ((scheck > car_s - 30) && (scheck < car_s + 30));               
            }
            else if (car_lane - lane == 1)
            {
              car_right = car_right | ((scheck > car_s - 30) && (scheck < car_s + 30));              
            }
          }
                    
          double speed_difference = 0;
          double max_speed = 49.5;
          double max_acc = 0.224;
          if (car_ahead)
          {
            if (!car_left && lane > 0)
            {
              lane--;              
            }
            else if (!car_right && lane != 2)
            {
              lane++;              
            }
            else
            {
              //std::cout << "car vel" << car_vel << " car_speed " << car_speed << std::endl;       
              if (ref_vel > car_vel)
              {                
                speed_difference -= max_acc*((ref_vel-car_vel)/ref_vel);
                //speed_difference -= max_acc;
              }
              else
              {
                //ref_vel = car_vel;
                speed_difference += max_acc*((car_vel-ref_vel)/car_vel);            
                //speed_difference += max_acc;
              }
            }
          }
          else
          {
            if (lane != 1)
            {
              if ((!car_right && lane == 0) || (!car_left && lane == 2))
              {
                lane = 1; //switch if on left lane/no car right or right lane/no car left                                
              }
            }
            if (ref_vel < max_speed)
            {
              speed_difference += max_acc;              
            }
          }
          
          //creating widely spaced points 
          vector<double> pointsx;
          vector<double> pointsy;

          //setting reference x, y, and yaw values of car
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw); 

          if (prev_size < 2)
          {
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);
            
            pointsx.push_back(prev_car_x);
            pointsx.push_back(car_x);
            
            pointsy.push_back(prev_car_y);
            pointsy.push_back(car_y);                       
          }
          else
          {
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];

            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y-ref_y_prev, ref_x - ref_x_prev);

            pointsx.push_back(ref_x_prev);
            pointsx.push_back(ref_x);

            pointsy.push_back(ref_y_prev);
            pointsy.push_back(ref_y);                        
          }
          
          
          
          vector<double> next_waypoint_0 = getXY(car_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_waypoint_1 = getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_waypoint_2 = getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

          pointsx.push_back(next_waypoint_0[0]);
          pointsx.push_back(next_waypoint_1[0]);
          pointsx.push_back(next_waypoint_2[0]);

          pointsy.push_back(next_waypoint_0[1]);
          pointsy.push_back(next_waypoint_1[1]);
          pointsy.push_back(next_waypoint_2[1]);
          
          //doing a local transformation so that car is at 0 degrees- simplifies the math
          for (int i = 0; i < pointsx.size(); i++)
          {
            double shiftx = pointsx[i] - ref_x;
            double shifty = pointsy[i] - ref_y;
            
            pointsx[i] = (shiftx * cos(0 - ref_yaw) - shifty * sin(0 - ref_yaw));
            pointsy[i] = (shiftx * sin(0 - ref_yaw) + shifty * cos(0 - ref_yaw));
            
          }
          
          //finally using the spline function to connect all the x and y points
          tk::spline s;
          
          s.set_points(pointsx, pointsy);
          
          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          
          for (int i = 0; i < previous_path_x.size(); i++)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          
          double targetx = 30;
          double targety = s(targetx);
          double targetdistance = sqrt(targetx*targetx + targety*targety);
          
          double xaddon = 0;
          
          for (int i = 1; i <= 50 - previous_path_x.size(); i++)
          {
            ref_vel += speed_difference;
            if (ref_vel > max_speed)
            {
              ref_vel = max_speed;              
            }        
            else if (ref_vel < max_acc)
            {
              ref_vel = max_acc;              
            }
            
            double N = (targetdistance/(0.02*ref_vel/2.24));
            double xpoint = xaddon + targetx/N;
            double ypoint = s(xpoint);
            
            xaddon = xpoint;
            
            double x_ref = xpoint;
            double y_ref = ypoint;
            
            xpoint = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
            ypoint = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));
            
            xpoint += ref_x;
            ypoint += ref_y;
            
            next_x_vals.push_back(xpoint);
            next_y_vals.push_back(ypoint);
            
          }
          
          

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
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
