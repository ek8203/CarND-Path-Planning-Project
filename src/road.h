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

	int update_width = 70;

  	string ego_rep = " *** ";

  	int ego_key = -1;

  	int num_lanes;

    vector<double> lane_speeds;

    double speed_limit;

    double density;

    int camera_center;

    map<int, Vehicle> vehicles;

    Vehicle ego;

    int vehicles_added = 0;

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

    void update_traffic(map<int, Vehicle> & vehicles);

  	void populate_traffic();

    void advance_ego(int prev_size);
    void advance();

    void  display();
  	void display(int timestep);

  	void update_ego(double x, double y, double s, double d, double yaw, double speed);

  	void update_ego(Vehicle vehicle);

    void add_ego(int lane_num, int s, vector<double> config_data);

  	void cull();

};

#endif /* ROAD_H_ */
