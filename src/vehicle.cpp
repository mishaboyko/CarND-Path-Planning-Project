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

bool Vehicle::is_vehicle_ahead(){
    bool vehicle_detected = false;

    // Basic structure for a vehicle ahead.
    // Rationale: it may happen, that the vehicle ahead of us from the sensor list is actually not the closes one due to cutting-in vehicles from other lanes.
    // The structure is: ID, s distance from ego-vehicle to that vehicle.
    vector<double> vehicle_ahead;

    double ego_s_value;

    /**
     * If there is a path for a vehicle to drive -
     * start Leading vehicle detection from the end of this path (ideally MAX_HORIZON points ahead of the vehicle).
     */
    if (rest_path_x.size() > 0){
        ego_s_value = end_path_s;
    }

    for(size_t i = 0; i < sensor_fusion.size(); ++i){
        float vehicle_d = sensor_fusion[i][6];
        // check whether car is in my lane (doesn't matter how far it is)
        if(vehicle_d < (OFFSET_FROM_LANE_MID+4 * curr_lane+OFFSET_FROM_LANE_MID) && vehicle_d > (OFFSET_FROM_LANE_MID+4 * curr_lane-OFFSET_FROM_LANE_MID)){
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];

            // Trigonometry, Euclidean distance.
            double leading_vehicle_speed = sqrt(vx*vx + vy*vy);
            leading_vehicle_speed *= MPH_MPS_factor;

            double leading_vehicle_s = sensor_fusion[i][5];
            /** Extrapolate the S-position of the leading vehicle.
             * Rationale: This position is a sensor data snapshot from previous cycle. And the algorithm processes (ideally) MAX_HORIZON points ahead.
             */
            //ego_s_value += (double)rest_path_x.size()*0.02 * leading_vehicle_speed;

            // react on leading vehicle
            if(leading_vehicle_s > ego_s_value && (leading_vehicle_s - ego_s_value) < SAFETY_DISTANCE_AHEAD){
                // std::cout << "Vehicle ahead detected with speed " << leading_vehicle_speed << std::endl;
                vehicle_detected = true;

                if(vehicle_ahead.empty() || vehicle_ahead[1] > (leading_vehicle_s - ego_s_value)){
                    // handle use-case the car is cutting-in, so it is closer that 30 points ahead of us or not the first in the list of vehicles.
                    vehicle_ahead = {static_cast<double>(i), (leading_vehicle_s - ego_s_value)};
                    // Slow down and switch to convoying mode.
                    target_velocity = leading_vehicle_speed;
                    // std::cout << "Slowing down to " << target_velocity << std::endl;
                }
            }
        }
    }
    return vehicle_detected;
}

void Vehicle::analyze_surrounding(){
    // sort detected vehicles into corresponding lanes.
    // Vehicles have attributes: map[ID]: [0] = s position, [1] = absolute velocity
    map<int, vector<double>> vehicles_in_lane0_ahead;
    map<int, vector<double>> vehicles_in_lane0_behind;
    map<int, vector<double>> vehicles_in_lane1_ahead;
    map<int, vector<double>> vehicles_in_lane1_behind;
    map<int, vector<double>> vehicles_in_lane2_ahead;
    map<int, vector<double>> vehicles_in_lane2_behind;

    for(size_t i = 0; i < sensor_fusion.size(); ++i){
        double vehicle_s = sensor_fusion[i][5];
        double vehicle_d = sensor_fusion[i][6];

        double vx = sensor_fusion[i][3];
        double vy = sensor_fusion[i][4];
        // Trigonometry, Euclidean distance.
        double vehicle_speed = sqrt(vx*vx + vy*vy) * MPH_MPS_factor;

        // check whether car is in my lane (doesn't matter how far it is)
        if(vehicle_d < (OFFSET_FROM_LANE_MID * 2) && vehicle_d > 0){
            // Lane 0
            if(vehicle_s < car_s){
                vehicles_in_lane0_behind.insert(std::pair<int, vector<double>>(i, {vehicle_s, vehicle_speed}));
            }
            else{
                vehicles_in_lane0_ahead.insert(std::pair<int, vector<double>>(i, {vehicle_s, vehicle_speed}));
            }
        } else if(vehicle_d < (4* OFFSET_FROM_LANE_MID) && vehicle_d > (2*OFFSET_FROM_LANE_MID)){
            // Lane 1
            if(vehicle_s < car_s){
                vehicles_in_lane1_behind.insert(std::pair<int, vector<double>>(i, {vehicle_s, vehicle_speed}));
            }
            else{
                vehicles_in_lane1_ahead.insert(std::pair<int, vector<double>>(i, {vehicle_s, vehicle_speed}));
            }
        } else{
            // Lane 2;
            if(vehicle_s < car_s){
                vehicles_in_lane2_behind.insert(std::pair<int, vector<double>>(i, {vehicle_s, vehicle_speed}));
            }
            else{
                vehicles_in_lane2_ahead.insert(std::pair<int, vector<double>>(i, {vehicle_s, vehicle_speed}));
            }
        }
    }
    // aggregate maps for simplicity
    std::vector<map<int, vector<double>>> lanes_ahead = {vehicles_in_lane0_ahead, vehicles_in_lane1_ahead, vehicles_in_lane2_ahead};
    std::vector<map<int, vector<double>>> lanes_behind = {vehicles_in_lane0_behind, vehicles_in_lane1_behind, vehicles_in_lane2_behind};

    // std::cout << vehicles_in_lane0_ahead.size() << " | " << vehicles_in_lane1_ahead.size() << " | " << vehicles_in_lane2_ahead.size() << std::endl;
    // std::cout << car_s << std::endl;
    // std::cout << vehicles_in_lane0_behind.size() << " | " << vehicles_in_lane1_behind.size()<< " | " << vehicles_in_lane2_behind.size() << std::endl;


    /**
     * Find most appropriate lane to change.
     * Criteria:
     * Prio 1: Free ahead of us in that lane.
     * Prio 2: the vehicle is far ahead of us in that lane and is faster than in ego-lane and other lane.
     * 2. Enough distance to the vehicle behind in the pre-selected lane for a LC Maneuver.
     */
    int best_lane_ahead = curr_lane;

    // re-use vehicle iterator for vehicles ahead and behind ego-vehicle
    map<int, vector<double>>::iterator vehicle_it;

    // Prio 1: Free ahead of us in that lane.
    for (size_t i = 0; i < lanes_ahead.size(); ++i){
        if(lanes_ahead[i].empty()){
            best_lane_ahead = i;
            // std::cout << "Prio 1 Best lane: " << i << std::endl;
        }
    }

    // no perfect lane based on Prio 1.
    if (best_lane_ahead == curr_lane){
        // Prio 2: the vehicle is far ahead of us in that lane and is faster than in ego-lane and other lane.
        double max_best_dist_ahead = 0;
        double max_best_velocity_ahead = 0;

        for (size_t i = 0; i < lanes_ahead.size(); ++i){
            int v_num;
            double min_dist = 9999;
            for(vehicle_it = lanes_ahead[i].begin(); vehicle_it != lanes_ahead[i].end(); ++vehicle_it){
                // get the closest vehicle to the ego-vehicle in that lane as a reference
                double relative_dist = vehicle_it->second[0] - car_s;
                if ( relative_dist < min_dist){
                    min_dist = relative_dist;
                    v_num = vehicle_it->first;
                }
            }

            if(min_dist > SAFETY_DISTANCE_AHEAD && lanes_ahead[i][v_num][1] >= car_speed){
                // among all lanes, get the one where preceding vehicle is further ahead
                if (max_best_dist_ahead < min_dist){
                    best_lane_ahead = i;
                    // std::cout << "Prio 2:  Lane "<< best_lane_ahead <<": min_dist "<< min_dist << std::endl;
                    max_best_dist_ahead = min_dist;
                    max_best_velocity_ahead = lanes_ahead[i][v_num][1];
                }
            }
        }
    }

    // if one of the lanes (other than ego-lane) is perfect, check vehicles behind ego-vehicle in that lane
    if (best_lane_ahead != curr_lane){
        int v_num;
        double min_dist = 9999;
        bool rear_clearance = false;
        bool mid_lane_clearance = false;

        for(vehicle_it = lanes_behind[best_lane_ahead].begin(); vehicle_it != lanes_behind[best_lane_ahead].end(); vehicle_it++){
            // get the closest vehicle to the ego-vehicle in that lane as a reference
            double relative_dist = car_s - vehicle_it->second[0];
            if ( relative_dist < min_dist){
                min_dist = relative_dist;
                v_num = vehicle_it->first;
            }
        }
        // std::cout << "Closest vehicle in best lane from behind: " << v_num << " min_dist " << min_dist << std::endl;

        /** Ignore the speed of the vehicle behind. Reasons:
         * 1. Sufficient safety distance.
         * 2. Ego-vehicle will immediately speed up to the MAX vehicle, which is >= of the vehicle behind.
         */
        if(min_dist >= SAFETY_DISTANCE_BEHIND){
            // std::cout << "Criteria 2 fulfilled:  Lane "<< best_lane_ahead <<" is the best lane" << std::endl;
            rear_clearance = true;
        }

        // Handle double lane-change to avoid stucking in the outter lanes.
        if(std::abs(curr_lane - best_lane_ahead) > 1){
            // std::cout << "Attempting to perform double lane change!" << std::endl;
            double min_dist_ahead = 9999;
            double min_dist_behind = 9999;

            for(vehicle_it = lanes_ahead[1].begin(); vehicle_it != lanes_ahead[1].end(); ++vehicle_it){
                // get the closest vehicle to the ego-vehicle in that lane as a reference
                double relative_dist = vehicle_it->second[0] - car_s;
                if ( relative_dist < min_dist_ahead){
                    min_dist_ahead = relative_dist;
                }
            }

            for(vehicle_it = lanes_behind[1].begin(); vehicle_it != lanes_behind[1].end(); ++vehicle_it){
                // get the closest vehicle to the ego-vehicle in that lane as a reference
                double relative_dist = car_s - vehicle_it->second[0];
                if (relative_dist < min_dist_behind){
                    min_dist_behind = relative_dist;
                    v_num = vehicle_it->first;
                }
            }

            if(min_dist_ahead > SAFETY_DISTANCE_AHEAD/2 && min_dist_behind > SAFETY_DISTANCE_BEHIND){
                mid_lane_clearance = true;
            }
        }
        else {
            mid_lane_clearance = true;
        }

        // 3. Trigger LC{R|L}
        if(rear_clearance && mid_lane_clearance){
            curr_lane = best_lane_ahead;
        }
    }
}

void Vehicle::look_ahead(){
    bool leading_vehicle_detected = is_vehicle_ahead();

    if(leading_vehicle_detected){
        if(current_velocity > target_velocity && car_speed > target_velocity ){
        current_velocity -= MAX_ACC;
        }
        analyze_surrounding();
    }
    else{
        /** Set target vehicle speed to MAX and speed up if:
        *  - there's no vehicle ahead (anymore).
        *  - Ego vehicle is slower than the MAX speed.
        */
        target_velocity = 49.5;

        if(current_velocity < target_velocity && car_speed < target_velocity){
            current_velocity += MAX_ACC;
            // std::cout << "Accelerating to V MAX " << car_speed << std::endl;
        }
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
    vector<double> next_wp0 = helper.getXY(car_s + SAMPLE_PTS_DIST, OFFSET_FROM_LANE_MID + 4*curr_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp1 = helper.getXY(car_s + (SAMPLE_PTS_DIST*2), OFFSET_FROM_LANE_MID + 4*curr_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp2 = helper.getXY(car_s + (SAMPLE_PTS_DIST*3), OFFSET_FROM_LANE_MID + 4*curr_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

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
