#ifndef VEHICLE_H
#define VEHICLE_H
#include "constant.h"

class Vehicle {

public:
  int id;   // car unique identifier
  int l;    // car lane
  double s; // car longitudinal distance m
  double d; // car lateral distance m
  double v; // car speed m/s
  double a; // car acceleration m/(s*s)

  STATE state = KL;

    struct collider
    {
      bool collision ; // is there a collision?
      double  time; // time collision happens
      collider(bool m_collision = false, double m_time = INF):collision(m_collision), time(m_time){};
    } collider;

  /**
  * Constructor
  */
  Vehicle();
  Vehicle(int id, int l, double s, double d, double v, double a, STATE state = KL);

  /**
  * Destructor
  */
  virtual ~Vehicle();
};

#endif