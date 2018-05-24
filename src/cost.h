/*
 * cost.h
 *
 *  Created on: May 6, 2018
 *      Author: nmkekrop
 */

#ifndef COST_H_
#define COST_H_

#include "vehicle.h"

using namespace std;

double calculate_cost(const Vehicle & vehicle, const map<int, vector<Vehicle>> & predictions, const vector<Vehicle> & trajectory);

//float distance_cost(const Vehicle & vehicle,  const vector<Vehicle> & trajectory,  const map<int, vector<Vehicle>> & predictions, map<string, float> & data);

//float inefficiency_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions, map<string, float> & data);

float lane_speed(const map<int, vector<Vehicle>> & predictions, int lane);

map<string, float> get_helper_data(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions);

#endif /* COST_H_ */
