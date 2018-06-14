#include <math.h>
#include <map>
#include <vector>
#include <string>
#include "constant.h"
#include "vehicle.h"
#include "cost.h"

using namespace std;

map<STATE, int> LANE_DIRECTION = {{PLCL, -1}, {LCL, -1}, {LCR, 1}, {PLCR, 1}};

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

string enum2str(STATE state)
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
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  if (closestWaypoint == maps_x.size())
  {
    closestWaypoint = 0;
  }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

//Get lane from lateral offset d
int get_lane(double d){
  return d/LANE_WIDTH;
}

//get d from lane
double get_d(int lane){
	return lane*4.+2.;
}

//convert miles per hour to metres per second
double mph2mps(double mph)
{
	return mph * MPH2MPS;
}

//convert metres per second to miles per hour
double mps2mph(double mps)
{
	return mps * MPS2MPH;
}


/*
	Retrieve vehicle from sensor fusion data within FOV.
*/
map<int, Vehicle> get_vehicle_in_FOV(vector<vector<double>> sensor_fusion, Vehicle ego)
{

	//vehicles within FOV
	map<int, Vehicle> vehicles;
	if(sensor_fusion.size() >= 0)
	{
		cout<<"empty sensor fusion data!"<<endl;
		return vehicles;
	}
	//calculate valid s range within FOV with warping of single round of track: 6945.554
	double ego_s = ego.s;
	double shift_s = .0; // shift vector: -1 warp backward; 0. no shift; 1. warp forward

	if(ego_s + FOV > MAX_S)//warp backward
	{
		shift_s = MAX_S - (ego_s + FOV);
	}
	else if(ego_s - FOV < 0.0) //warp forward
	{
		shift_s = FOV - ego_s;
	}
	//end of calculate 

	//shift ego vehicle s
	double shifted_s = ego_s + shift_s;

	for(int i=0; i< sensor_fusion.size();i++)
	{
	  //shift sensor fusion vehicle with shift_s
	  double s = sensor_fusion[i][5]; 
	  double s_ = fmod(s + shift_s + MAX_S, MAX_S);
	  //end of shifting

	  // filter vehicles beyond FOV: 70m
	  if(fabs(s_ - shifted_s) > FOV)
	  {
	    continue;
	  }
	  //end of s filtering

	  //filter vehicle out of normal lanes: 0, 1, 2( left . right)
	  double d = sensor_fusion[i][6];
	  int l = get_lane(d);
	  if( l > 2 || l < 0)
	  {
	    continue;
	  }
	  //end of lane filtering

	  //retrieve vehicle info
	  double id = sensor_fusion[i][0];
	  // double x = sensor_fusion[i][1];
	  // double y = sensor_fusion[i][2];
	  double vx = sensor_fusion[i][3];
	  double vy = sensor_fusion[i][4];
	  double speed = sqrt(vx*vx + vy*vy);

	  Vehicle vehicle = Vehicle(id, l, s, d, speed, 0, CS);
	  vehicles.insert(std::pair<int,Vehicle>(id,vehicle));

	  //debug vehicle within FOV
	  cout<<"id: "<<id<<"\ts: "<<s<<"\td: "<<d<<"\tspeed: "<<speed<<"\n";
	}
	cout<<"\n";

	return vehicles;
}


/* 
	Get predictions about these vehicles in horizon time span
*/
map<int, vector<Vehicle>> predict(map<int, Vehicle> vehicles, Vehicle ego)
{
	map<int, vector<Vehicle>> predictions;

	if(vehicles.size()<=0)
	{
		cout<<"empty FOV vehicles data!"<<endl;
		return predictions;
	}

	//parameters of ego vehicle
	int m_lane = ego.l;
	double m_v = ego.v;
	double m_s = ego.s;
	double m_d = ego.d;

	//prediction starts
	for(map<int, Vehicle>:: iterator it = vehicles.begin(); it != vehicles.end(); it++)
	{
		int id_ = it->first;
		Vehicle vehicle_ = it->second;

		int lane_ = vehicle_.l;
		double s_ = vehicle_.s;
		double d_ = vehicle_.d;
		double v_ = vehicle_.v;
		STATE state_ = vehicle_.state;

		vector<Vehicle> dt_vehicles;

		Vehicle d0_vehicle = Vehicle(id_, lane_, s_, d_, v_, .0, state_);
		dt_vehicles.push_back(d0_vehicle);

		for(double dt = .0; dt < HORIZON; dt+=DT)
		{

			double new_s = fmod(s_ + dt * v_, MAX_S);//calculate s between 0 to MAX_S
			Vehicle dt_v = Vehicle(id_, lane_, new_s, d_, v_, .0, state_);
			dt_vehicles.push_back(dt_v);
		}

		predictions.insert(std::pair<int, vector<Vehicle>>(id_, dt_vehicles));
	}

	return predictions;
}

vector<STATE> successor_states(Vehicle ego) {
    /*
    Provides the possible next states given the current state for the FSM 
    discussed in the course, with the exception that lane changes happen 
    instantaneously, so LCL and LCR can only transition back to KL.
    */
    vector<STATE> states;
    states.push_back(KL);
    STATE state = ego.state;
    int lane = ego.l;
    if(state == KL) 
    {
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

double position_at(double t, Vehicle m_vehicle) {
    return m_vehicle.s + mph2mps(m_vehicle.v * t) + m_vehicle.a * t * t/2.0;
}

Vehicle constant_speed_trajectory(Vehicle ego) {
    /*
    Generate a constant speed trajectory.
    */
    float next_pos = position_at(HORIZON, ego);
    Vehicle target = Vehicle(ego.id, ego.l, ego.s, ego.d, ego.v, ego.a, ego.state);
    return target;
}

bool get_vehicle_behind(map<int, vector<Vehicle>> predictions, int lane, Vehicle ego, Vehicle & rVehicle) {
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

bool get_vehicle_ahead(map<int, vector<Vehicle>> predictions, int lane, Vehicle ego, Vehicle & rVehicle) {
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

vector<double> get_kinematics(map<int, vector<Vehicle>> predictions, int lane, Vehicle ego) {
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

  if (get_vehicle_ahead(predictions, lane, ego, vehicle_ahead)) {
      // cout<<"\nfind vehicle ahead!\n";

      if (get_vehicle_behind(predictions, lane, ego, vehicle_behind)) {
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

Vehicle keep_lane_trajectory(map<int, vector<Vehicle>> predictions, Vehicle ego) {
    /*
    Generate a keep lane trajectory.
    */
  
    Vehicle target;

    vector<double> kinematics = get_kinematics(predictions, ego.l, ego);
    float new_s = kinematics[0];
    float new_v = kinematics[1];
    float new_a = kinematics[2];
    cout<<"keep lane:\ts: "<<new_s<<"\tv: "<<new_v<<"\ta: "<<new_a<<endl;
    target = Vehicle(ego.id, ego.l, new_s, ego.d, new_v, new_a, KL);
    
    return target;
}

Vehicle prep_lane_change_trajectory(STATE state, map<int, vector<Vehicle>> predictions, Vehicle ego) {

  /*
  Generate a trajectory preparing for a lane change.
  */
  float new_s;
  float new_v;
  float new_a;
  Vehicle vehicle_behind;
  int new_lane = ego.l + LANE_DIRECTION[state];
  Vehicle target;

  vector<double> curr_lane_new_kinematics = get_kinematics(predictions, ego.l, ego);

  if (get_vehicle_behind(predictions, ego.l, ego, vehicle_behind)) {
      //Keep speed of current lane so as not to collide with car behind.
      new_s = curr_lane_new_kinematics[0];
      new_v = curr_lane_new_kinematics[1];
      new_a = curr_lane_new_kinematics[2];
      
  } else {
      vector<double> best_kinematics;
      vector<double> next_lane_new_kinematics = get_kinematics(predictions, new_lane, ego);
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
  cout<<"prepare lane change:\ts: "<<new_s<<"\tv: "<<new_v<<"\ta: "<<new_a<<"\tnew_lane: "<<new_lane<<endl;
  target = Vehicle(ego.id, ego.l, new_s, ego.d, new_v, new_a, state);
  return target;
}

Vehicle lane_change_trajectory(STATE state, map<int, vector<Vehicle>> predictions, Vehicle ego) 
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
        return target;
      }
      // else
      // {
      //     lane_changeable = true;
      // }
  }
	
	vector<double> kinematics = get_kinematics(predictions, new_lane, ego);
	cout<<"lane change:\ts: "<<kinematics[0]<<"\tv: "<<kinematics[1]<<"\ta: "<<kinematics[2]<<"\tnew_lane: "<<new_lane<<endl;
	target = Vehicle(ego.id, new_lane, kinematics[0], get_d(new_lane), kinematics[1], kinematics[2], state);
  
  return target;
}


vector<Vehicle> generate_target(STATE state, map<int ,vector<Vehicle>> predictions, Vehicle ego)
{
	// map<STATE, Vehicle> targets;	
	//get possible target vehicle
	// for(int i=0; i<states.size(); i++)
	// {
		// STATE state = states[i];

		vector<Vehicle> targets;
		Vehicle target;

		switch(state)
		{
			case CS:
							target = constant_speed_trajectory(ego);
							break;
			case KL:
							target = keep_lane_trajectory(predictions, ego);
							break;
			case PLCL:
							target = prep_lane_change_trajectory(PLCL, predictions, ego);
								break;
			case PLCR:
							target = prep_lane_change_trajectory(PLCR, predictions, ego);
								break;
			case LCL:
							target = lane_change_trajectory(LCL, predictions, ego);
								break;
			case LCR: 
								target = lane_change_trajectory(LCR, predictions, ego);
								break;
			default: cout<<"bad state";
							 break;
			if(target.id > -1)
			{
				targets.push_back(target);
			}
		}
	// }
	return targets;
}

//
bool check_collision(map<int ,vector<Vehicle>> predictions, Vehicle ego)
{
	//TODO
	return false;
}

/*
	get target vehicle at horizon from predicitons
*/
Vehicle get_target_vehicle(map<int ,vector<Vehicle>> predictions, Vehicle ego)
{
	//check collision
	// if(! check_collision(predictions, ego))
	// {
		//get possible next state list
		vector<STATE> states = successor_states(ego);
		cout<<"states size: "<<states.size()<<"\n";
		assert(states.size()>1);

		//get possible target vechile
		// map<STATE, Vehicle> targets = generate_target(states, predictions, ego);

		//calculate cost for each state
		float cost;
		vector<float> costs;
    vector<string> final_states;
    vector<Vehicle> final_trajectories;

    for (vector<STATE>::iterator it = states.begin(); it != states.end(); ++it) {
        vector<Vehicle> trajectory = generate_target(*it, predictions, ego);
        assert(trajectory.size() != 0);
        if (trajectory.size() != 0) {
            cost = calculate_cost(ego, predictions, trajectory);
            cout<<"cost for: "<<enum2str(*it)<<"\t: "<<cost<<"\n";
            costs.push_back(cost);
            final_trajectories.push_back(trajectory[0]);
        }
    }

    vector<float>::iterator best_cost = min_element(begin(costs), end(costs));
    int best_idx = distance(begin(costs), best_cost);
    return final_trajectories[best_idx];

		//choose best state	
	// }
	// else
	// {
	// 	//deal with collison
	// 	//TODO 
	// 	//deal with collision
	// 	//keep lane if there is possible collision
	// 	Vehicle target = keep_lane_trajectory(predictions, ego);
	// 	return target;
	// }
	
}
