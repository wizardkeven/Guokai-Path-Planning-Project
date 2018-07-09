#include "cost.h"
#include "vehicle.h"
#include <functional>
#include <iterator>
#include <map>
#include <math.h>
#include "constant.h"

const double REACH_GOAL = pow(10, 2);
const double EFFICIENCY = pow(10, 4);
const double LANE_SPEED = pow(10, 1);


double goal_distance_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions, map<string, double> & data) {
    /*
    Cost increases based on distance of intended lane (for planning a lane change) and final lane of trajectory.
    Cost of being out of goal lane also becomes larger as vehicle approaches goal distance.
    */
    double cost;
    double distance = data["distance_to_goal"];
    if (distance > 0) {
        cost = 1 - exp(-(fabs(data["intended_lane"] - data["final_lane"]) / distance));
    } else {
        cost = 1;
    }
    return cost;
}

double inefficiency_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions, map<string, double> & data) {
    /*
    Cost becomes higher for trajectories with intended lane and final lane that have traffic slower than vehicle's target speed. 
    */

    double proposed_speed_intended = lane_speed(predictions, data["intended_lane"]);
    if (proposed_speed_intended < 0) {
        proposed_speed_intended = SPEED_LIMIT;
    }

    double proposed_speed_final = lane_speed(predictions, data["final_lane"]);
    if (proposed_speed_final < 0) {
        proposed_speed_final = SPEED_LIMIT;
    }
    
    double cost = (2.0*SPEED_LIMIT - proposed_speed_intended - proposed_speed_final)/SPEED_LIMIT;

    return cost;
}


double lane_speed(const map<int, vector<Vehicle>> & predictions, int lane) {
    /*
    All non ego vehicles in a lane have the same speed, so to get the speed limit for a lane,
    we can just find one vehicle in that lane.
    */
    for (map<int, vector<Vehicle>>::const_iterator it = predictions.begin(); it != predictions.end(); ++it) {
        int key = it->first;
        Vehicle vehicle = it->second[0];
        if (vehicle.l == lane && key != -1) {
            return vehicle.v;
        }
    }
    //Found no vehicle in the lane
    return -1.0;
}

double calculate_cost(const Vehicle & vehicle, const map<int, vector<Vehicle>> & predictions, const vector<Vehicle> & trajectory) {
    /*
    Sum weighted cost functions to get total cost for trajectory.
    */
    map<string, double> trajectory_data = get_helper_data(vehicle, trajectory, predictions);
    double cost = 0.0;

    //Add additional cost functions here.
    vector< function<double(const Vehicle & , const vector<Vehicle> &, const map<int, vector<Vehicle>> &, map<string, double> &)>> cf_list = {goal_distance_cost, inefficiency_cost};
    vector<double> weight_list = {REACH_GOAL, EFFICIENCY};
    
    for (int i = 0; i < cf_list.size(); i++) {
        double new_cost = weight_list[i]*cf_list[i](vehicle, trajectory, predictions, trajectory_data);
        cost += new_cost;
    }

    return cost;

}

map<string, double> get_helper_data(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions) {
    /*
    Generate helper data to use in cost functions:
    indended_lane: the current lane +/- 1 if vehicle is planning or executing a lane change.
    final_lane: the lane of the vehicle at the end of the trajectory.
    distance_to_goal: the distance of the vehicle to the goal.

    Note that indended_lane and final_lane are both included to help differentiate between planning and executing
    a lane change in the cost functions.
    */
    map<string, double> trajectory_data;
    Vehicle trajectory_last = trajectory[0];
    double intended_lane;

    if (trajectory_last.state == PLCL) {
        intended_lane = trajectory_last.l - 1;
    } else if (trajectory_last.state == PLCR) {
        intended_lane = trajectory_last.l + 1;
    } else {
        intended_lane = trajectory_last.l;
    }

    double distance_to_goal = MAX_S*4 - trajectory_last.s;
    double final_lane = trajectory_last.l;
    double final_d = trajectory_last.d;
    double final_v = trajectory_last.v;
    double final_a = trajectory_last.a;

    trajectory_data["intended_lane"] = intended_lane;
    trajectory_data["final_lane"] = final_lane;
    trajectory_data["final_d"] = final_d;
    trajectory_data["final_v"] = final_v;
    trajectory_data["final_a"] = final_a;
    trajectory_data["distance_to_goal"] = distance_to_goal;
    return trajectory_data;
}

