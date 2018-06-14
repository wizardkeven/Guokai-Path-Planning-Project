#include "vehicle.h"

Vehicle::Vehicle()
{
		this->id = -1;
    this->l = 1;
    this->s = .0;
    this->d = .0;
    this->v = .0;
    this->a = .0;
    this->state = CS;
}

Vehicle::Vehicle(int id, int l, double s, double d, double v, double a, STATE state){

		this->id = id;
    this->l = l;
    this->s = s;
    this->d = d;
    this->v = v;
    this->a = a;
    this->state = state;
}

Vehicle::~Vehicle() {}