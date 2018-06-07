#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <random>
#include <vector>
#include <map>
#include "constant.h"

using namespace std;

class Vehicle {
public:

  map<STATE, int> LANE_DIRECTION = {{PLCL, -1}, {LCL, -1}, {LCR, 1}, {PLCR, 1}};
  struct collider{

    bool collision ; // is there a collision?
    double  time; // time collision happens
    collider(bool m_collision = false, double m_time = SCLT):collision(m_collision), time(m_time){};

  } collider;

  int lane;

  double s;

  double v;

  double a;

  double target_speed;

  STATE state;


  /**
  * Constructor
  */
  Vehicle();
  Vehicle(int lane, double s, double v, double a, STATE state = CS);

  /**
  * Destructor
  */
  virtual ~Vehicle();

  vector<vector<double>> choose_next_state(map<int, vector<Vehicle>> predictions);

  vector<STATE> successor_states();

  vector<Vehicle> generate_trajectory(STATE state, map<int, vector<Vehicle>> predictions);

  vector<double> get_kinematics(map<int, vector<Vehicle>> predictions, int lane, int n_wp);

  vector<Vehicle> constant_speed_trajectory();

  vector<Vehicle> keep_lane_trajectory(map<int, vector<Vehicle>> predictions);

  vector<Vehicle> lane_change_trajectory(STATE state, map<int, vector<Vehicle>> predictions);

  vector<Vehicle> prep_lane_change_trajectory(STATE state, map<int, vector<Vehicle>> predictions);

  double position_at(double t);

  bool get_vehicle_behind(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle, int n_wp);

  bool get_vehicle_ahead(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle, int n_wp);

  vector<Vehicle> generate_predictions();

};

#endif