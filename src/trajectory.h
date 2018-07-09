#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <vector>
#include <map>
#include <vector>
#include <string>
#include "constant.h"
#include "vehicle.h"
#include "cost.h"


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
	std::map<string(*)(), int> WEIGHTED_COST_FUNCTIONS;

	std::vector<traj_xy> traj;
	/**
  * Constructor
  */
	Trajectory(Vehicle start_vehicle, Vehicle target_vehicle, std::vector<double> delta, std::vector<std::vector<double>> previous_xy);
	/**
  * Destructor
  */
	virtual ~Trajectory();

	std::vector<std::vector<double>> generate_trajectory();

};

#endif