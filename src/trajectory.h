#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <vector>
#include <map>
#include <vector>
#include <string>
#include "constant.h"
#include "vehicle.h"
#include "cost.h"

using namespace std;

class Trajectory{

public:
	struct traj_frenet{
		double s;
		double d;
	};

	struct traj_xy
	{	
		double x;
		double y;
	};

	//existing cost functions
	map<string(*)(), int> WEIGHTED_COST_FUNCTIONS;

	vector<traj_xy> traj;
	/**
  * Constructor
  */
	Trajectory(Vehicle start_vehicle, Vehicle target_vehicle, vector<double> delta, vector<vector<double>> previous_xy);
	/**
  * Destructor
  */
	virtual ~Trajectory();

	vector<vector<double>> generate_trajectory();

};

#endif