#include "target.h"

using namespace std;

Target::Target(map<int ,vector<Vehicle> > predictions, Vehicle ego)
{
  this->predictions = predictions;
  this->ego = ego;
}

Target::~Target(){}


string Target::enum2str(STATE state)
{

  switch(state)
  {
    case CS: return "CS";   // constant speed
    case KL: return "KL";   // keep lane
    case PLCL: return "PLCL"; //prepare lane change left
    case PLCR: return "PLCR"; //prepare lane change right
    case LCL: return "LCL";  //lane change left
    case LCR: return "LCR"; //Lane change right
    default: return "Unknown state!"; 
  }
}

void Target::successor_states(vector<STATE>& rStates) {
    /*
    Provides the possible next states given the current state for the FSM 
    discussed in the course, with the exception that lane changes happen 
    instantaneously, so LCL and LCR can only transition back to KL.
    */
    rStates.emplace_back(ego.state);
    STATE state = ego.state;
    int lane = ego.l;
    if(state == KL) 
    {
      rStates.emplace_back(PLCL);
      rStates.emplace_back(PLCR);
    } else if (state == PLCL) {
      if (lane != 0) {
          rStates.emplace_back(PLCL);
          rStates.emplace_back(LCL);
      }
    } else if (state == PLCR) {
      if (lane != NUM_LANE - 1) {
          rStates.emplace_back(PLCR);
          rStates.emplace_back(LCR);
      }
    }
}


double Target::position_at(double t, Vehicle m_vehicle) {
    return m_vehicle.s + mph2mps(m_vehicle.v * t) + m_vehicle.a * t * t/2.0;
}

void Target::constant_speed_trajectory(Vehicle & rVehicle) {
    /*
    Generate a constant speed trajectory.
    */
    float next_pos = position_at(HORIZON, ego);
    Vehicle target = Vehicle(ego.id, ego.l, ego.s, ego.d, ego.v, ego.a, ego.state);
    rVehicle = target;
    // return target;
}

bool Target::get_vehicle_behind(int lane, Vehicle & rVehicle) {
    /*
    Returns a true if a vehicle is found behind the current vehicle, false otherwise. The passed reference
    rVehicle is updated if a vehicle is found.
    */
    double max_s = -1;
    bool found_vehicle = false;
    Vehicle temp_vehicle;
    for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
        
        temp_vehicle = it->second[it->second.size()-1];//otherwise take the last waypoint
        double future_ego_s = fmod(position_at(HORIZON, ego), MAX_S);
        if (temp_vehicle.l == lane && temp_vehicle.s < future_ego_s && temp_vehicle.s > max_s) {
            max_s = temp_vehicle.s;
            rVehicle = temp_vehicle;
            found_vehicle = true;
        }
    }
    return found_vehicle;
}

bool Target::get_vehicle_ahead(int lane, Vehicle & rVehicle) {
    /*
    Returns a true if a vehicle is found ahead of the current vehicle, false otherwise. The passed reference
    rVehicle is updated if a vehicle is found.
    */
    double min_s = MAX_S;
    bool found_vehicle = false;
    Vehicle temp_vehicle;
    for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {

        temp_vehicle = it->second[it->second.size()-1];// take last waypoint
        double future_ego_s = fmod(position_at(HORIZON, ego), MAX_S);
        if (temp_vehicle.l == lane && temp_vehicle.s > future_ego_s && temp_vehicle.s < min_s) {
            min_s = temp_vehicle.s;
            rVehicle = temp_vehicle;
            found_vehicle = true;
        }
    }
    return found_vehicle;
}

vector<double> Target::get_kinematics(int lane) {
  /* 
  Gets next timestep kinematics (position, velocity, acceleration) 
  for a given lane. Tries to choose the maximum velocity and acceleration, 
  given other vehicle positions and accel/velocity constraints.
  */

  double m_v = mph2mps(ego.v);//convert to m/s for easy calculation
  double m_s = ego.s;
  double m_a = ego.a;

  double max_velocity_accel_limit = m_v + MAX_ACC * 0.9 * HORIZON;//we plan trajectory for HORIZON also for HORIZON
  double new_position;
  double new_velocity;
  double new_accel;
  Vehicle vehicle_ahead;
  Vehicle vehicle_behind;

  if (get_vehicle_ahead(lane, vehicle_ahead)) {
      // cout<<"\nfind vehicle ahead!\n";

      if (get_vehicle_behind(lane, vehicle_behind)) {
          new_velocity = vehicle_ahead.v; //must travel at the speed of traffic, regardless of preferred buffer
      } else {
          double max_velocity_in_front = (vehicle_ahead.s - m_s - SAFE_DIST - CAR_L)/HORIZON + vehicle_ahead.v - HORIZON * m_a / 2.0;
          new_velocity = min(min(max_velocity_in_front, max_velocity_accel_limit), SPEED_LIMIT);
      }
  } else {
      // cout<<"\ndidn't find vehicle ahead!\n";
      new_velocity = min(max_velocity_accel_limit, SPEED_LIMIT);
  }
  
  new_accel = (new_velocity - m_v)/HORIZON; //Equation: (v_1 - v_0)/t = acceleration
  new_velocity = mps2mph(new_velocity);
  new_position = m_s + HORIZON * new_velocity + HORIZON * new_accel / 2.0;
  return{new_position, new_velocity, new_accel};
    
}

void Target::keep_lane_trajectory(Vehicle &rVehicle) {
    /*
    Generate a keep lane trajectory.
    */
  
    Vehicle target;

    vector<double> kinematics = get_kinematics(ego.l);
    float new_s = kinematics[0];
    float new_v = kinematics[1];
    float new_a = kinematics[2];
    // cout<<"keep lane:\ts: "<<new_s<<"\tv: "<<new_v<<"\ta: "<<new_a<<"\tnew_lane: "<<ego.l<<endl;
    target = Vehicle(ego.id, ego.l, new_s, ego.d, new_v, new_a, KL);
    rVehicle = target;
    
    // return true;
}

void Target::prep_lane_change_trajectory(STATE state, Vehicle &rVehicle) {

  /*
  Generate a trajectory preparing for a lane change.
  */
  float new_s;
  float new_v;
  float new_a;
  Vehicle vehicle_behind;
  int new_lane = ego.l + LANE_DIRECTION[state];
  Vehicle target;

  vector<double> curr_lane_new_kinematics = get_kinematics(ego.l);

  if (get_vehicle_behind(ego.l, vehicle_behind)) {
      //Keep speed of current lane so as not to collide with car behind.
      new_s = curr_lane_new_kinematics[0];
      new_v = curr_lane_new_kinematics[1];
      new_a = curr_lane_new_kinematics[2];
      
  } else {
      vector<double> best_kinematics;
      vector<double> next_lane_new_kinematics = get_kinematics(new_lane);
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
  // cout<<"prepare lane change:\ts: "<<new_s<<"\tv: "<<new_v<<"\ta: "<<new_a<<"\tnew_lane: "<<new_lane<<endl;
  target = Vehicle(ego.id, ego.l, new_s, ego.d, new_v, new_a, state);
  rVehicle = target;
  // return true;
}


bool Target::lane_change_trajectory(STATE state, Vehicle &rVehicle) 
{
  /*
  Generate a lane change trajectory.
  */
  int new_lane = ego.l + LANE_DIRECTION[state];
  Vehicle target;
  Vehicle next_lane_vehicle;
  // bool lane_changeable = false;
  //Check if a lane change is possible (check if another vehicle occupies that spot).
  for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
      next_lane_vehicle = it->second[it->second.size() -1];
      vector<double> kinematics;
      double lc_dist = fabs(fmod(next_lane_vehicle.s, MAX_S) - fmod(ego.s + SAFE_DIST + CAR_L, MAX_S));
      if ( lc_dist > 0 && next_lane_vehicle.l == new_lane) {
        //If lane change is not possible, return empty trajectory.
        return false;
      }
      // else
      // {
      //     lane_changeable = true;
      // }
  }
  
  vector<double> kinematics = get_kinematics(new_lane);
  // cout<<"lane change:\ts: "<<kinematics[0]<<"\tv: "<<kinematics[1]<<"\ta: "<<kinematics[2]<<"\tnew_lane: "<<new_lane<<endl;
  target = Vehicle(ego.id, new_lane, kinematics[0], get_d(new_lane), kinematics[1], kinematics[2], state);
  rVehicle = target;
  
  return true;
}


bool Target::generate_target(const STATE& state, vector<Vehicle>& rTargets)
{
  // map<STATE, Vehicle> targets; 
  //get possible target vehicle
  // for(int i=0; i<states.size(); i++)
  // {
    // STATE state = states[i];
    // targets.push_back(ego);
    Vehicle target;
    bool traj_valid = false;
    switch(state)
    {
      case CS:
              // cout<<"generate target for: CS\n";
              constant_speed_trajectory(target);
              traj_valid = true;
              break;
      case KL:
              // cout<<"generate target for: KL\n";
              keep_lane_trajectory(target);
              traj_valid = true;
              break;
      case PLCL:
              // cout<<"generate target for: PLCL\n";
              prep_lane_change_trajectory(PLCL, target);
              traj_valid = true;
              break;
      case PLCR:
              // cout<<"generate target for: PLCR\n";
              prep_lane_change_trajectory(PLCR, target);
              traj_valid = true;
              break;
      case LCL:
              // cout<<"generate target for: LCL\n";
              traj_valid = lane_change_trajectory(LCL, target);
              break;
      case LCR:
              // cout<<"generate target for: LCR\n"; 
              traj_valid = lane_change_trajectory(LCR, target);
              break;
      default: cout<<"bad state";
              break;
    }
    if(traj_valid)
    {
      rTargets.emplace_back(target);
    }
  // }
  return traj_valid;
}

//
bool Target::check_collision()
{
  //TODO  
  return false;
}

/*
  get target vehicle at horizon from predicitons
*/
void Target::get_target_vehicle(Vehicle& rVehicle)
{
  //check collision
  // if(! check_collision(predictions, ego))
  // {
    //get possible next state list
    vector<STATE> states;
    successor_states(states);
    // cout<<"states size: "<<states.size()<<"\n";
    // assert(states.size()>1);

    //get possible target vechile
    // map<STATE, Vehicle> targets = generate_target(states, predictions, ego);

    //calculate cost for each state
    double cost;
    vector<double> costs;
    vector<STATE> traj_states;
    vector<string> final_states;
    vector<Vehicle> final_trajectories;
    Vehicle finale_vehicel;

    for (vector<STATE>::iterator it = states.begin(); it != states.end(); ++it) {
        vector<Vehicle> trajectory;
        cost = INF;
        bool traj_valid = generate_target(*it, trajectory);
        if(traj_valid)
        {
          // assert(trajectory.size() != 0);
          if (trajectory.size() != 0) {
              cost = calculate_cost(ego, predictions, trajectory);
              cout<<"cost for: "<<enum2str(*it)<<"\t: "<<cost<<"\n";
              costs.push_back(cost);
              traj_states.push_back(*it);
              final_trajectories.push_back(trajectory[0]);
          }
        }else{
          cout<<"no valid trajectory for state: "<<*it<<"\n";
        }
        cout<<"\n";
    }

    vector<double>::iterator best_cost = min_element(begin(costs), end(costs));
    int best_idx = distance(begin(costs), best_cost);
    cout<<"\nFrom state: "<<enum2str(ego.state)<<" to state: "<<enum2str(traj_states[best_idx])<<"\n";
    rVehicle = move(final_trajectories[best_idx]);

    //choose best state 
  // }
  // else
  // {
  //  //deal with collison
  //  //TODO 
  //  //deal with collision
  //  //keep lane if there is possible collision
  //  Vehicle target = keep_lane_trajectory(predictions, ego);
  //  return target;
  // }
  
}
