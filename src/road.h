/*
 * road.h
 *
 *  Created on: May 6, 2018
 *      Author: nmkekrop
 */

#ifndef ROAD_H_
#define ROAD_H_

#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <set>
#include <map>
#include <string>
#include <iterator>
#include "vehicle.h"
#include "json.hpp"

using namespace std;

class Road {
public:

  	int ego_key = -1;

  	int num_lanes;

    double speed_limit;

    map<int, Vehicle> vehicles;

    Vehicle ego;

    /**
  	* Constructor
  	*/
    Road();

    Road(double speed_limit, int num_lanes);

  	/**
  	* Destructor
  	*/
  	virtual ~Road();

  	Vehicle get_ego();

    void update_traffic(vector<Vehicle> & traffic);

    void update_ego(Vehicle vehicle);

    void advance_ego(int prev_size);

    void  display();

  	void cull();

};

#endif /* ROAD_H_ */
