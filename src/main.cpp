#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "spline.h"
#include "json.hpp"

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


    // start in lane 1 [0, 1, 2] from middle to curb.
    int curr_lane = 1;

    const int OFFSET_FROM_LANE_MID = 2;

    // because double solid line is 0 and we're in the middle of the 2nd lane and the lane width is 4m (4+2=6)
    double next_d = OFFSET_FROM_LANE_MID+4 * curr_lane;

    // Set to Max. allowed velocity first (49.5 mph)
    double target_velocity = 49.5;

    double current_velocity = 0.01;

    const double MAX_HORIZON = 50;

    const double SAMPLE_PTS_DIST = 30.f; //in m

    const double MPH_MPS_factor = 2.24;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy, &OFFSET_FROM_LANE_MID,
              &curr_lane, &next_d, &target_velocity, &current_velocity, &MAX_HORIZON, &SAMPLE_PTS_DIST, &MPH_MPS_factor]
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

          // Path, that had been previously calculated and given to the Planner, but hasn't been driven yet.
          auto rest_path_x = j[1]["previous_path_x"];
          auto rest_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          /**
           * Sensor Fusion Data, a list of all other cars on the same side of the road.
           * 2D vector of cars, where 2nd is a car-specific:
           * [ unique ID,
           * X-position Cartesinan, Y-position Cartesian,
           * X velocity (m/s), Y velocity (m/s),
           * S-position Frenet, D-position Frenet ]
           */
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          vector<double> next_s_coords;
          vector<double> next_d_coords;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

          /**
           * Create a list of widely spaced (x, y) waypoints, evenly spaced at 30m distance from each other.
           * Then connect these points by more points, which are interpolated using an spline.
           */
          vector<double> pts_x;
          vector<double> pts_y;

          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          /** Leading car detection
           * If there is a path for a vehicle to drive -
           * start Leading vehicle detection from the end of this path (ideally MAX_HORIZON points ahead of the vehicle).
           */
          double reference_s_value;
          if (rest_path_x.size() > 0){
            reference_s_value = end_path_s;
          }

          bool leading_vehicle_detected = false;

          for(size_t i = 0; i < sensor_fusion.size(); ++i){
            float leading_vehicle_d = sensor_fusion[i][6];
            // check whether car is in my lane
            //if(leading_vehicle_d < next_d + OFFSET_FROM_LANE_MID && leading_vehicle_d > next_d - OFFSET_FROM_LANE_MID){
            if(leading_vehicle_d < (next_d+OFFSET_FROM_LANE_MID) && leading_vehicle_d > (next_d-OFFSET_FROM_LANE_MID)){
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];

              // Trigonometry, euclidean distance.
              double leading_vehicle_speed = sqrt(vx*vx + vy*vy);
              double leading_vehicle_s = sensor_fusion[i][5];

              /** Extrapolate the S-position of the leading vehicle.
               * Rationale: This position is a sensor data snapshot from previous cycle. And the algorithm processes (ideally) MAX_HORIZON points ahead.
               */
              //reference_s_value += (double)rest_path_x.size()*0.02 * leading_vehicle_speed;

              // react on leading vehicle
              if(leading_vehicle_s > reference_s_value && (leading_vehicle_s - reference_s_value) < 10){
                std::cout << "Leading vehicle detected. Slowing down to target velocity" << std::endl;
                target_velocity = leading_vehicle_speed * MPH_MPS_factor;
                leading_vehicle_detected = true;
              }
            }
          }
          // better idea: go through each reference point and change the velocity there.
          if(leading_vehicle_detected && current_velocity > target_velocity && car_speed > target_velocity ){
            // equals approximately to 5 m/s^2 acceleration
            current_velocity -= 0.224;
          }

          /** Set target vehicle spped to MAX if:
           *  - there's no vehicle ahead (anymore).
           *  - We are speeding up (e.g. simulation start)
           */
          else if(!leading_vehicle_detected && current_velocity < target_velocity && car_speed < target_velocity){
            target_velocity = 49.5;
            // equals approximately to 5 m/s^2 acceleration
            current_velocity += 0.224;
            std::cout << car_speed << std::endl;
            std::cout << "Speeding up to MAX target_velocity" << std::endl;
          }

          if (rest_path_x.size() < 2){

            // Use the two points, that make the path tangent to the car


            // extrapolate the point at t-1 into past to build a tangent.
            pts_x.push_back(ref_x - cos(ref_yaw));
            pts_y.push_back(ref_y - sin(ref_yaw));

            pts_x.push_back(ref_x);
            pts_y.push_back(ref_y);
          }
          // Use the end of the previously calculated path as a starting point for further extention of the horizon.
          else {
            ref_x = rest_path_x[rest_path_x.size() - 1];
            double prev_ref_x = rest_path_x[rest_path_x.size() - 2];

            ref_y = rest_path_y[rest_path_x.size() - 1];
            double prev_ref_y = rest_path_y[rest_path_x.size() - 2];

            // important: atan2 instead of atan and (y,x) coords, not (x,y)
            ref_yaw = atan2(ref_y - prev_ref_y, ref_x - prev_ref_x);

            // Use the two points, that make the path tangent to the previously calculated path's endpoints
            pts_x.push_back(prev_ref_x);
            pts_x.push_back(ref_x);

            pts_y.push_back(prev_ref_y);
            pts_y.push_back(ref_y);
          }

          // In Freenet add evelty spaced points (30m distance) ahead of the starting reference
          vector<double> next_wp0 = getXY(car_s + SAMPLE_PTS_DIST, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s + (SAMPLE_PTS_DIST*2), next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s + (SAMPLE_PTS_DIST*3), next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);

          pts_x.push_back(next_wp0[0]);
          pts_x.push_back(next_wp1[0]);
          pts_x.push_back(next_wp2[0]);

          pts_y.push_back(next_wp0[1]);
          pts_y.push_back(next_wp1[1]);
          pts_y.push_back(next_wp2[1]);

          // for all coordinates of the waypoints, shift car reference angle to 0 degrees
          for (size_t i = 0; i < pts_x.size(); ++i){
            double shift_x = pts_x[i] - ref_x;
            double shift_y = pts_y[i] - ref_y;

            pts_x[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
            pts_y[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));

          }

          // create a spline
          tk::spline spline;

          spline.set_points(pts_x, pts_y);

          // re-use the points from the previously calculated trajectory
          for(size_t i = 0; i < rest_path_x.size(); ++i){
            next_x_vals.push_back(rest_path_x[i]);
            next_y_vals.push_back(rest_path_y[i]);
          }

          // Number of sections into which we split the spline. This defines the velocity, since a simulator takes 1 point per 0.2 sec.
          // d = N_sections * time * vel (50 * 0.2 * ~50mph)
          double target_x = SAMPLE_PTS_DIST;
          double target_y = spline(target_x);
          double target_dist = sqrt(target_x*target_x + target_y*target_y);

          double x_add_on = 0;

          // fill the rest of the path planner
          for (size_t i = 1; i < MAX_HORIZON - rest_path_x.size(); ++i) {
            double N = target_dist / (0.02*current_velocity / MPH_MPS_factor);
            double x_point = x_add_on + (target_x/N);
            double y_point = spline(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            // rotate back to cartesian coordinates (undo rotation to Frenet coordinates)
            x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
            y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

            x_point += ref_x;
            y_point += ref_y;


            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
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