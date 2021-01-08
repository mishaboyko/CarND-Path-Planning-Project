#ifndef VEHICLE_H
#define VEHICLE_H

#include <iostream>
#include <vector>
#include "json.hpp"
#include "helpers.h"
#include "spline.h"

using nlohmann::json;
using namespace std;

class Vehicle {
    public:
        Vehicle();
        virtual ~Vehicle();

        void setWaypoints(const vector<double> &map_waypoints_x, const vector<double> &map_waypoints_y, const vector<double> &map_waypoints_s,
                          const vector<double> &map_waypoints_dx, const vector<double> &map_waypoints_dy);

        void compute_next_points( vector<double> &next_x_vals, vector<double> &next_y_vals);

        void parseMessage(json j);

    private:
        void look_ahead();
        tk::spline extend_previous_path(vector<double> &pts_x, vector<double> &pts_y);

        vector<double> map_waypoints_x;
        vector<double> map_waypoints_y;
        vector<double> map_waypoints_s;
        vector<double> map_waypoints_dx;
        vector<double> map_waypoints_dy;

        // Data from JSON Message
        // Main car's localization Data
        double car_x;
        double car_y;
        double car_s;
        double car_d;
        double car_yaw;
        double car_speed;

        // Path, that had been previously calculated and given to the Planner, but hasn't been driven yet.
        json rest_path_x = json::array();
        json rest_path_y = json::array();

        // Previous path's end s and d values
        double end_path_s;
        double end_path_d;

        /**
        * Sensor Fusion Data, a list of all other cars on the same side of the road.
        * 2D vector of cars, where 2nd is a car-specific:
        * [ unique ID,
        * X-position Cartesinan, Y-position Cartesian,
        * X velocity (m/s), Y velocity (m/s),
        * S-position Frenet, D-position Frenet ]
        */
        json sensor_fusion = json::array();

        int curr_lane = 1; // lanes from middle to curb: [0, 1, 2]
        const int OFFSET_FROM_LANE_MID = 2;
        double target_velocity = 49.5; // in mph)
        double current_velocity = 0.01;
        const double MAX_HORIZON = 50;
        const double SAMPLE_PTS_DIST = 30.f; //in m
        const double MPH_MPS_factor = 2.24;

        // equals approximately to 2 * 5 m/s^2 acceleration
        const double MAX_ACC = .224*2;

        Helper helper;
};

#endif  // VEHICLE_H