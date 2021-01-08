#include "vehicle.h"

Vehicle::Vehicle(){}
Vehicle::~Vehicle() {}

void Vehicle::setWaypoints(const vector<double> &map_waypoints_x, const vector<double> &map_waypoints_y, const vector<double> &map_waypoints_s,
                          const vector<double> &map_waypoints_dx, const vector<double> &map_waypoints_dy){
    this->map_waypoints_x = map_waypoints_x;
    this->map_waypoints_y = map_waypoints_y;
    this->map_waypoints_s = map_waypoints_s;
    this->map_waypoints_dx = map_waypoints_dx;
    this->map_waypoints_dy = map_waypoints_dy;
}

void Vehicle::parseMessage(json j){
    // j[1] is the data JSON object

    // Main car's localization Data
    car_x = j[1]["x"];
    car_y = j[1]["y"];
    car_s = j[1]["s"];
    car_d = j[1]["d"];
    car_yaw = j[1]["yaw"];
    car_speed = j[1]["speed"];

    // Path, that had been previously calculated and given to the Planner, but hasn't been driven yet.
    rest_path_x = j[1]["previous_path_x"];
    rest_path_y = j[1]["previous_path_y"];
    // Previous path's end s and d values
    end_path_s = j[1]["end_path_s"];
    end_path_d = j[1]["end_path_d"];

    /**
    * Sensor Fusion Data, a list of all other cars on the same side of the road.
    * 2D vector of cars, where 2nd is a car-specific:
    * [ unique ID,
    * X-position Cartesinan,    Y-position Cartesian,
    * X velocity (m/s),         Y velocity (m/s),
    * S-position Frenet,        D-position Frenet ]
    */
    sensor_fusion = j[1]["sensor_fusion"];
}

void Vehicle::look_ahead(){
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
        if(leading_vehicle_d < (OFFSET_FROM_LANE_MID+4 * curr_lane+OFFSET_FROM_LANE_MID) && leading_vehicle_d > (OFFSET_FROM_LANE_MID+4 * curr_lane-OFFSET_FROM_LANE_MID)){
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
            if(leading_vehicle_s > reference_s_value && (leading_vehicle_s - reference_s_value) < 30){
                std::cout << "Leading vehicle detected. Slowing down to target velocity" << std::endl;
                target_velocity = leading_vehicle_speed * MPH_MPS_factor;
                leading_vehicle_detected = true;

                // TODO: 1. Handle PLC{R|L}
                // TODO: 2. Track all vehicles on the left and right
                // TODO: 3. Handle LC{R|L}
                if(curr_lane > 0){
                  //blindly turn left
                  curr_lane = 0;
                }
            }
            // TODO: 4. handle use-case the car is cutting-in, so it is closer that 30 points ahead of us. Currently ego_vehicle doesn't see cutting in car.
        }
    }
    if(leading_vehicle_detected && current_velocity > target_velocity && car_speed > target_velocity ){
        current_velocity -= MAX_ACC;
    }

    /** Set target vehicle speed to MAX if:
    *  - there's no vehicle ahead (anymore).
    *  - We are speeding up (e.g. simulation start)
    */
    else if(!leading_vehicle_detected && current_velocity < target_velocity && car_speed < target_velocity){
        target_velocity = 49.5;

        // equals approximately to 5 m/s^2 acceleration
        current_velocity += MAX_ACC;
        std::cout << car_speed << "mph. Speeding up to MAX target_velocity" << std::endl;
    }
}

tk::spline Vehicle::extend_previous_path(vector<double> &pts_x, vector<double> &pts_y) {
    // create a spline
    tk::spline spline;

    if (rest_path_x.size() < 2){
        // Use the two points, that make the path tangent to the car

        // extrapolate the point at t-1 into past to build a tangent.
        pts_x.push_back(car_x - cos(car_yaw));
        pts_y.push_back(car_y - sin(car_yaw));

        pts_x.push_back(car_x);
        pts_y.push_back(car_y);
    }
    // Use the end of the previously calculated path as a starting point for further extention of the horizon.
    else {
        car_x = rest_path_x[rest_path_x.size() - 1];
        double prev_car_x = rest_path_x[rest_path_x.size() - 2];

        car_y = rest_path_y[rest_path_x.size() - 1];
        double prev_car_y = rest_path_y[rest_path_x.size() - 2];

        // important: atan2 instead of atan and (y,x) coords, not (x,y)
        car_yaw = atan2(car_y - prev_car_y, car_x - prev_car_x);

        // Use the two points, that make the path tangent to the previously calculated path's endpoints
        pts_x.push_back(prev_car_x);
        pts_x.push_back(car_x);

        pts_y.push_back(prev_car_y);
        pts_y.push_back(car_y);
    }

    // In Freenet add evelty spaced points (30m distance) ahead of the starting reference
    // because double solid line is 0 and we're in the middle of the 2nd lane and the lane width is 4m (4+2=6)
    vector<double> next_wp0 = helper.getXY(car_s + SAMPLE_PTS_DIST, OFFSET_FROM_LANE_MID+4 * curr_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp1 = helper.getXY(car_s + (SAMPLE_PTS_DIST*2), OFFSET_FROM_LANE_MID+4 * curr_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp2 = helper.getXY(car_s + (SAMPLE_PTS_DIST*3), OFFSET_FROM_LANE_MID+4 * curr_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

    pts_x.push_back(next_wp0[0]);
    pts_x.push_back(next_wp1[0]);
    pts_x.push_back(next_wp2[0]);

    pts_y.push_back(next_wp0[1]);
    pts_y.push_back(next_wp1[1]);
    pts_y.push_back(next_wp2[1]);

    // for all coordinates of the waypoints, shift car reference angle to 0 degrees
    for (size_t i = 0; i < pts_x.size(); ++i){
        double shift_x = pts_x[i] - car_x;
        double shift_y = pts_y[i] - car_y;

        pts_x[i] = (shift_x * cos(0 - car_yaw) - shift_y * sin(0 - car_yaw));
        pts_y[i] = (shift_x * sin(0 - car_yaw) + shift_y * cos(0 - car_yaw));

    }

    spline.set_points(pts_x, pts_y);
    return spline;
}

void Vehicle::compute_next_points(vector<double> &next_x_vals, vector<double> &next_y_vals){
    /**
    * Create a list of widely spaced (x, y) waypoints, evenly spaced at 30m distance from each other.
    * Then connect these points by more points, which are interpolated using an spline.
    */
    vector<double> pts_x;
    vector<double> pts_y;

    car_yaw = helper.deg2rad(car_yaw);

    look_ahead();

    tk::spline spline = extend_previous_path(pts_x, pts_y);

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
        x_point = x_ref * cos(car_yaw) - y_ref * sin(car_yaw);
        y_point = x_ref * sin(car_yaw) + y_ref * cos(car_yaw);

        x_point += car_x;
        y_point += car_y;

        next_x_vals.push_back(x_point);
        next_y_vals.push_back(y_point);
    }
}
