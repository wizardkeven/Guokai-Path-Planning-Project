#include "prediction.h"

using namespace std;


Prediction::Prediction(const vector<vector<double>>& sensor_fusion, Vehicle& ego)
{
	this->sensor_fusion = sensor_fusion;
	this->ego = ego;
}

Prediction:: ~Prediction(){};

/*
	Retrieve vehicle from sensor fusion data within FOV.
*/
void Prediction::get_vehicle_in_FOV(std::map<int, Vehicle>& rVehicles)
{
	//vehicles within FOV
	if(sensor_fusion.size() >= 0)
	{
		// cout<<"empty sensor fusion data!"<<endl;
		return;
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
	  rVehicles.insert(std::pair<int,Vehicle>(id,vehicle));

	  //debug vehicle within FOV
	  cout<<"id: "<<id<<"\ts: "<<s<<"\td: "<<d<<"\tspeed: "<<speed<<"\n";
	}
	cout<<"\n";


}


/* 
	Get predictions about these vehicles in horizon time span
*/
void Prediction::predict(map<int ,vector<Vehicle>>& rPredictions)
{

	std::map<int, Vehicle> vehicles;
	//get vehicles from sensor fusion data within FOV
	get_vehicle_in_FOV(vehicles);

	if(vehicles.size()<=0)
	{
		// cout<<"empty FOV vehicles data!"<<endl;
		return;
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

		rPredictions.insert(std::pair<int, vector<Vehicle>>(id_, dt_vehicles));
	}
}