#ifndef TARGET_H
#define TARGET_H

#include <iostream>
#include <algorithm>
#include <vector>
#include <map>
#include <vector>
#include <string>
#include "constant.h"
#include "vehicle.h"
#include "cost.h"
// #include "helpers.h"

class Target{

public:
	Vehicle ego;
	std::map<int, std::vector<Vehicle>> predictions;
	std::map<STATE, int> LANE_DIRECTION = {{PLCL, -1}, {LCL, -1}, {LCR, 1}, {PLCR, 1}};

	/**
  * Constructor
  */
	Target(std::map<int ,std::vector<Vehicle> > predictions, Vehicle ego);
	/**
  * Destructor
  */
	virtual ~Target();

	std::string enum2str(STATE state);

	void successor_states(std::vector<STATE>& rStates);

	double position_at(double t, Vehicle m_vehicle);

	void constant_speed_trajectory(Vehicle& rVehicle) ;

	bool get_vehicle_behind(int lane,Vehicle& rVehicle);

	bool get_vehicle_ahead(int lane, Vehicle& rVehicle);

	std::vector<double> get_kinematics(int lane);

	void keep_lane_trajectory(Vehicle& rVehicle);

	void prep_lane_change_trajectory(STATE state, Vehicle& rVehicle);

	bool lane_change_trajectory(STATE state, Vehicle& rVehicle);

	bool generate_target(const STATE& state, std::vector<Vehicle>& rtargets);

	bool check_collision();

	void get_target_vehicle(Vehicle& rVehicle);

};

#endif