#include <algorithm>
#include <iostream>
#include "vehicle.h"
#include <cmath>
#include <map>
#include <string>
#include <iterator>
#include "cost.h"

/**
 * Initializes Vehicle
 */

Vehicle::Vehicle(){}

Vehicle::Vehicle(int lane, double s, double v, double a, STATE state) {

    this->lane = lane;
    this->s = s;
    this->v = v;
    this->a = a;
    this->state = state;
}

Vehicle::~Vehicle() {}


vector<vector<double>> Vehicle::choose_next_state(map<int, vector<Vehicle>> predictions) {
    /*
    
    ***Here you can implement the transition_function code from the Behavior Planning Pseudocode
    classroom concept.***
    
    INPUT: A predictions map. This is a map using vehicle id as keys with predicted
        vehicle trajectories as values. A trajectory is a vector of Vehicle objects. The first
        item in the trajectory represents the vehicle at the current timestep. The second item in
        the trajectory represents the vehicle one timestep in the future.
    OUTPUT: The the best (lowest cost) trajectory for the ego vehicle corresponding to the next ego vehicle state.

    Functions that will be useful:
    1. successor_states() - Uses the current state to return a vector of possible successor states for the finite 
       state machine.
    2. generate_trajectory(string state, map<int, vector<Vehicle>> predictions) - Returns a vector of Vehicle objects 
       representing a vehicle trajectory, given a state and predictions. Note that trajectory vectors 
       might have size 0 if no possible trajectory exists for the state. 
    3. calculate_cost(Vehicle vehicle, map<int, vector<Vehicle>> predictions, vector<Vehicle> trajectory) - Included from 
       cost.cpp, computes the cost for a trajectory.
    */
    
    //TODO: Your solution here.
    map<STATE, float> costs;
    vector<STATE> possible_successor_states = successor_states();
    map<STATE, vector<Vehicle>> next_states;
    for(int i = 0; i < possible_successor_states.size(); i++)
    {
        STATE possible_successor_state = possible_successor_states[i];
        vector<Vehicle> trajectory_for_state = generate_trajectory(possible_successor_states[i], predictions);
        next_states.insert(pair<STATE, vector<Vehicle>>(possible_successor_state,trajectory_for_state));
        float cost_for_state = 0.0;
        if(trajectory_for_state.size() > 0)
        {
            cost_for_state = calculate_cost(*this, predictions, trajectory_for_state);
        }
        costs.insert(pair<STATE, float>(possible_successor_state , cost_for_state));
        
    }
    
    vector<vector<double>> best_next_state;
    float min_cost = 9999999;
    for(map<STATE, vector<Vehicle>>::iterator it = next_states.begin(); it != next_states.end(); ++it) 
    {
        STATE state = it->first;
        float cost = costs.at(state);
        if(cost < min_cost)
        {
            min_cost = cost;
            Vehicle m_vehicle = it->second;
            best_next_state.push_back({m_vehicle.s, m_vehicle.lane});
        }
    }
    //TODO: Change return value here:
    return best_next_state;
}

vector<STATE> Vehicle::successor_states() {
    /*
    Provides the possible next states given the current state for the FSM 
    discussed in the course, with the exception that lane changes happen 
    instantaneously, so LCL and LCR can only transition back to KL.
    */
    vector<STATE> states;
    states.push_back(KL);
    STATE state = this->state;
    if(state == KL) {
        states.push_back(PLCL);
        states.push_back(PLCR);
    } else if (state == PLCL) {
        if (lane != 0) {
            states.push_back(PLCL);
            states.push_back(LCL);
        }
    } else if (state == PLCR) {
        if (lane != NUM_LANE - 1) {
            states.push_back(PLCR);
            states.push_back(LCR);
        }
    }
    //If state is "LCL" or "LCR", then just return "KL"
    return states;
}

vector<Vehicle> Vehicle::generate_trajectory(STATE state, map<int, vector<Vehicle>> predictions) {
    /*
    Given a possible next state, generate the appropriate trajectory to realize the next state.
    */
    vector<Vehicle> trajectory;
    if (state == CS) {
        trajectory = constant_speed_trajectory();
    } else if (state == KL) {
        trajectory = keep_lane_trajectory(predictions);
    } else if (state ==LCL || state ==LCR ) {
        trajectory = lane_change_trajectory(state, predictions);
    } else if (state == PLCL || state == PLCR ) {
        trajectory = prep_lane_change_trajectory(state, predictions);
    }
    return trajectory;
}

vector<double> Vehicle::get_kinematics(map<int, vector<Vehicle>> predictions, int lane, int n_wp) {
    /* 
    Gets next timestep kinematics (position, velocity, acceleration) 
    for a given lane. Tries to choose the maximum velocity and acceleration, 
    given other vehicle positions and accel/velocity constraints.
    */
    double max_velocity_accel_limit = MAX_ACC * 0.9  + this->v;
    double new_position;
    double new_velocity;
    double new_accel;
    Vehicle vehicle_ahead;
    Vehicle vehicle_behind;
    double delta_t = n_wp * DT;

    if (get_vehicle_ahead(predictions, lane, vehicle_ahead, n_wp)) {
        // cout<<"\nfind vehicle ahead!\n";

        if (get_vehicle_behind(predictions, lane, vehicle_behind, n_wp)) {
            new_velocity = vehicle_ahead.v; //must travel at the speed of traffic, regardless of preferred buffer
        } else {
            double max_velocity_in_front = (vehicle_ahead.s - this->s - SAFE_DIST)/delta_t + vehicle_ahead.v - delta_t * 0.5 * (this->a);
            new_velocity = min(min(max_velocity_in_front, max_velocity_accel_limit), this->target_speed);
        }
    } else {
        // cout<<"\ndidn't find vehicle ahead!\n";
        new_velocity = min(max_velocity_accel_limit, this->target_speed);
    }
    
    new_accel = (new_velocity - this->v)/delta_t; //Equation: (v_1 - v_0)/t = acceleration
    new_position = this->s + delta_t * new_velocity + delta_t * new_accel / 2.0;
    return{new_position, new_velocity, new_accel};
    
}

vector<Vehicle> Vehicle::constant_speed_trajectory() {
    /*
    Generate a constant speed trajectory.
    */
    //total waypoints number
    int ctrl_n = HORIZON/DT;
    
    vector<Vehicle> trajectory = {Vehicle(this->lane, this->s, this->v, this->a, this->state)};
    for(int i=1; i<ctrl_n; i++)
    {
        float next_pos = position_at(i * DT);
        Vehicle next_wp = Vehicle(this->lane, next_pos, this->v, 0, this->state);
        trajectory.push_back(next_wp);
    }
    return trajectory;
}

vector<Vehicle> Vehicle::keep_lane_trajectory(map<int, vector<Vehicle>> predictions) {
    /*
    Generate a keep lane trajectory.
    */
    //total waypoints number
    int ctrl_n = HORIZON/DT;
    vector<Vehicle> trajectory = {Vehicle(lane, this->s, this->v, this->a, state)};

    for(int i=1; i<ctrl_n; i++)
    {
        vector<double> kinematics = get_kinematics(predictions, this->lane, i);
        float new_s = kinematics[0];
        float new_v = kinematics[1];
        float new_a = kinematics[2];
        trajectory.push_back(Vehicle(this->lane, new_s, new_v, new_a, KL));
    }
    
    return trajectory;
}

vector<Vehicle> Vehicle::prep_lane_change_trajectory(STATE state, map<int, vector<Vehicle>> predictions) {
    /*
    Generate a trajectory preparing for a lane change.
    */
    //total waypoints number
    int ctrl_n = HORIZON/DT;
    float new_s;
    float new_v;
    float new_a;
    Vehicle vehicle_behind;
    double s_l_c_
    int new_lane = this->lane + LANE_DIRECTION[state];
    vector<Vehicle> trajectory = {Vehicle(this->lane, this->s, this->v, this->a, this->state)};

    for(int i=1; i<ctrl_n; i++)
    {
        vector<double> curr_lane_new_kinematics = get_kinematics(predictions, this->lane, i);

        if (get_vehicle_behind(predictions, this->lane, vehicle_behind, i)) {
            //Keep speed of current lane so as not to collide with car behind.
            new_s = curr_lane_new_kinematics[0];
            new_v = curr_lane_new_kinematics[1];
            new_a = curr_lane_new_kinematics[2];
            
        } else {
            vector<double> best_kinematics;
            vector<double> next_lane_new_kinematics = get_kinematics(predictions, new_lane, i);
            //Choose kinematics with lowest velocity.
            if (next_lane_new_kinematics[1] < curr_lane_new_kinematics[1]) {
                best_kinematics = next_lane_new_kinematics;
            } else {
                best_kinematics = curr_lane_new_kinematics;
            }
            new_s = best_kinematics[0];
            new_v = best_kinematics[1];
            new_a = best_kinematics[2];
        }

        trajectory.push_back(Vehicle(this->lane, new_s, new_v, new_a, state));
    }
    return trajectory;
}

vector<Vehicle> Vehicle::lane_change_trajectory(STATE state, map<int, vector<Vehicle>> predictions) {
    /*
    Generate a lane change trajectory.
    */
    //total waypoints number
    int ctrl_n = HORIZON/DT;
    int new_lane = this->lane + LANE_DIRECTION[state];
    vector<Vehicle> trajectory;
    Vehicle next_lane_vehicle;
    bool lane_changeable = false;

    trajectory.push_back(Vehicle(this->lane, this->s, this->v, this->a, this->state));
    for(int i=1; i<ctrl_n; i++)
    {
        //Check if a lane change is possible (check if another vehicle occupies that spot).
        for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
            next_lane_vehicle = it->second[0];
            vector<double> kinematics;
            if (next_lane_vehicle.s == this->s && next_lane_vehicle.lane == new_lane) {
                // //If lane change is not possible, return empty trajectory.
                // return trajectory;
                //if lane change is not possible for the current timestamp, just keep lane
                kinematics = get_kinematics(predictions, this->lane, i);
            }else
            {
                lane_changeable = true;
                kinematics = get_kinematics(predictions, new_lane, i);
            }
            trajectory.push_back(Vehicle(new_lane, kinematics[0], kinematics[1], kinematics[2], state));
        }
    
    }
    //if lane change is not possible through all time slots, return empty trajectory
    if(!lane_changeable){
        vector<Vehicle> no_trajectory;
        return no_trajectory;
    }
    
    return trajectory;
}

double Vehicle::position_at(double t) {
    return this->s + this->v*t + this->a*t*t/2.0;
}

bool Vehicle::get_vehicle_behind(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle, int n_wp) {
    /*
    Returns a true if a vehicle is found behind the current vehicle, false otherwise. The passed reference
    rVehicle is updated if a vehicle is found.
    */
    double max_s = -1;
    bool found_vehicle = false;
    Vehicle temp_vehicle;
    for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
        if(it->second.size() >= n_wp)
        {
            temp_vehicle = it->second[n_wp];//take the n-th waypoint
        }else
        {
            temp_vehicle = it->second[it->second.size()-1];//otherwise take the last waypoint
        }
        double delta_t = n_wp* DT;//future timestamp
        if (temp_vehicle.lane == this->lane && temp_vehicle.s < position_at(delta_t) && temp_vehicle.s > max_s) {
            max_s = temp_vehicle.s;
            rVehicle = temp_vehicle;
            found_vehicle = true;
        }
    }
    return found_vehicle;
}

bool Vehicle::get_vehicle_ahead(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle, int n_wp) {
    /*
    Returns a true if a vehicle is found ahead of the current vehicle, false otherwise. The passed reference
    rVehicle is updated if a vehicle is found.
    */
    double min_s = GOAL_S;
    bool found_vehicle = false;
    Vehicle temp_vehicle;
    for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
        if(it->second.size() >= n_wp)
        {
            temp_vehicle = it->second[n_wp];//take the n-th waypoint
        }else
        {
            temp_vehicle = it->second[it->second.size()-1];//otherwise take the last waypoint
        }
        double delta_t = n_wp* DT;//future timestamp
        if (temp_vehicle.lane == this->lane && temp_vehicle.s > position_at(delta_t) && temp_vehicle.s < min_s) {
            min_s = temp_vehicle.s;
            rVehicle = temp_vehicle;
            found_vehicle = true;
        }
    }
    return found_vehicle;
}

vector<Vehicle> Vehicle::generate_predictions() {
    /*
    Generates predictions for non-ego vehicles to be used
    in trajectory generation for the ego vehicle.
    */
	vector<Vehicle> predictions;
    double current_v = this->v;
    double delta_t = DT;

    for(double i = delta_t; i < HORIZON; i+= delta_t) {
      double next_s = position_at(i);
      double next_v = current_v;
      if (i < HORIZON - delta_t) {
        next_v = (position_at(i+delta_t) - next_s)/delta_t;
      }
      current_v = next_v;
      predictions.push_back(Vehicle(this->lane, next_s, next_v, 0));
  	}
    return predictions;
}