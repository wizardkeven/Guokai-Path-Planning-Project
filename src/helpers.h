#include <math.h>
#include <map>
#include <vector>
#include <string>
#include "constant.h"
#include "vehicle.h"

using namespace std;

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
map<int, Vehicle> get_vehicle_in_FOV(vector<double> sensor_fusion, Vehicle ego)
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
	double shift_s = .0; // shift vector: -1 warp backward; 0-> no shift; 1-> warp forward

	if(ego_s + FOV > MAX_S)//warp backward
	{
		shift_s = MAX_X - (ego_s + FOV);
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

	  //filter vehicle out of normal lanes: 0, 1, 2( left -> right)
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

	  Vehicle vehicle = Vehicle(id, l, s, d, lane_speed, 0, CS);
	  vehicles.insert(std::pair<int,Vehicle>(id,vehicle));

	  //debug vehicle within FOV
	  cout<<"id: "<<id<<"\ts: "<<s<<"\td: "<<d<<"\tlane_speed: "<<lane_speed<<"\n";
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
	for(map<int, Vehicle>:: Iterator it = vehicles.begin(); it != vehicles.end(); it++)
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

		for(double dt = .0; dt < horizon; dt+=DT)
		{

			double new_s = s_ + dt * v_;
			Vehicle dt_v = Vehicle(id_, lane_, new_s, d_, v_, .0, state_);
			dt_vehicles.push_back(dt_v);
		}

		predictions.insert(std::pair<int, vector<Vehicle>>(id_, dt_vehicles));
	}

	return predictions;
}

vector<STATE> successor_states(Vehicle & ego) {
    /*
    Provides the possible next states given the current state for the FSM 
    discussed in the course, with the exception that lane changes happen 
    instantaneously, so LCL and LCR can only transition back to KL.
    */
    vector<STATE> states;
    states.push_back(KL)
    STATE state = ego->state;
    int lane = ego->l;
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


map<STATE, Vehicle> generate_target(vector<STATE> states, map<int ,vector<Vehicle>> predictions, Vehicle &ego)
{
	map<STATE, Vehicle> targets;	
	//get possible target vehicle
	for(int i=0; i<states.size(); i++)
	{
		STATE state = states[i];
		STATE ego_state = ego->state;

		Vehicle target;
		switch(ego_state)
		{
			case CS:
							target = constant_trajectory(&ego);
							break;
			case KL:
							target = keep_lane_trajectory(vector<STATE> states, map<int ,vector<Vehicle>> predictions, Vehicle &ego);
							break;
			case PLCL:
							//TODO
								break;
			case PLCR:
								//TODO
								break;
			case LCL:
								//TODO
								break;
			case LCR: 
								//TODO
								break;
			default: cout<<"bad state";
							 break;
			if(! target)
			{
				targets.insert(std::pair<state, target>);
			}
		}
	}
}

/*
	get target vehicle at horizon from predicitons
*/
Vehicle get_target_vehicle(map<int ,vector<Vehicle>> predictions, Vehicle &ego)
{
	//get possible next state list
	vector<STATE> states = successor_states(&ego);

	//get possible target vechile
	map<STATE, Vehicle> targets = generate_target(states, predictions, &ego);

	//calculate cost for each state
	//TODO

	//choose best state
}
