#ifndef PREDICTION_H
#define PREDICTION_H
#include <iostream>
#include <map>
#include <math.h>
#include <string>
#include <vector>
#include "constant.h"
#include "vehicle.h"
// #include "helpers.h"

class Prediction {

public:

  /**
  * Constructor
  */
  Prediction(const std::vector<std::vector<double>>& sensor_fusion, Vehicle& ego);

  /**
  * Destructor
  */
  virtual ~Prediction();
  
  void get_vehicle_in_FOV(std::map<int, Vehicle>& rVehicles);

  void predict(std::map<int ,std::vector<Vehicle>>& rPredictions);

private:
  std::vector<std::vector<double>> sensor_fusion;
  Vehicle ego;
};

#endif