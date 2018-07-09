#ifndef COST_H
#define COST_H

#include <map>
#include <string>
#include <vector>
#include "vehicle.h"
#include <vector>

using namespace std;

double calculate_cost(const Vehicle & vehicle, const map<int, vector<Vehicle>> & predictions, const vector<Vehicle> & trajectory);

double goal_distance_cost(const Vehicle & vehicle,  const vector<Vehicle> & trajectory,  const map<int, vector<Vehicle>> & predictions, map<string, double> & data);

double inefficiency_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions, map<string, double> & data);

double lane_speed(const map<int, vector<Vehicle>> & predictions, int lane);

map<string, double> get_helper_data(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions);

#endif