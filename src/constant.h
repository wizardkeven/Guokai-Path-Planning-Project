#ifndef CONSTANT_H
#define CONSTANT_H

const double INF = 1e10;

//start in lane 1
const int START_LANE = 1;
const double MAX_ACC = 10.; // m/s_2
const double MAX_JERK = 10.; // m/s_2
const int L = 1; //car inherent property
const double HORIZON = 1.0;//for three time slots
const int LANE_WIDTH = 4;
const double SPEED_LIMIT = 49.5;
const double NUM_LANE = 3;
const double GOAL_S = 1000000; //m
const double GOAL_LANE = 3;
const double MAX_ACCEL_FACTOR = .224;
 // The max s value before wrapping around the track back to 0
const double MAX_S = 6945.554;//single round length of track
const double DT = .02; // s; output point time slot
const double CAR_W = 4.; //safe width of car
const double CAR_L = 6.; //safe length of car
const int SAFE_DIST = 6; // impacts "keep lane" behavior.
const double FOV = 50.; //field of view with which the objects will be taken into consideration
const double SCLT = 0.53; //s, shortest change lane time, based on shortest time of reaching lateral edge lane line 
const double MPH2MPS = .447; //unit conversion coefficient from miles per hour to metres per second
const double MPS2MPH = 2.237; //unit conversion coefficient from metres per second to miles per hour 
enum STATE
{
  CS,   // constant speed
  KL,   // keep lane
  PLCL, //prepare lane change left
  PLCR, //prepare lane change right
  LCL,  //lane change left
  LCR   //lane change right
};


#endif