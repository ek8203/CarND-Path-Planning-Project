/*
 * road.cpp
 *
 *  Created on: May 6, 2018
 *      Author: nmkekrop
 */

#include <iostream>
#include "road.h"
#include "vehicle.h"
#include <math.h>
#include <map>
#include <string>
#include <iterator>
#include "json.hpp"


/**
 * Initializes Road
 */
Road::Road(double speed_limit, int num_lanes) {
  this->num_lanes = num_lanes;
  this->speed_limit = speed_limit;

  ego_key   = -1;
  ego = Vehicle(ego_key, 1, 0, 0, 0);
  ego.state = "KL";
  ego.too_close = false;
}

Road::Road() {
  num_lanes = 3;
  speed_limit = 49.5;
  ego_key   = -1;
  ego = Vehicle(ego_key, 1, 0, 0, 0);
  ego.state = "KL";
  ego.too_close = false;
}

Road::~Road() {}

Vehicle Road::get_ego() {

  return this->ego;
}

void Road::update_traffic(vector<Vehicle> & traffic) {

  // Clear traffic
  this->vehicles.clear();

  for(int i = 0; i < traffic.size(); i++)  {
    // Update traffic
    vehicles.insert(std::pair<int,Vehicle>(i, traffic[i]));
  }
}


void Road::display()  {

  map<int, Vehicle>::iterator it = this->vehicles.begin();

  while(it != this->vehicles.end()) {
    Vehicle vehicle = it->second;
    cout << it->first << ":" << vehicle.id << " -> " << vehicle.s << " lane " << vehicle.lane << " state " << vehicle.state << endl;
    it++;
  }
}

void Road::advance_ego(int prev_size) {

  map<int ,vector<Vehicle> > predictions;
  map<int, Vehicle>::iterator it = this->vehicles.begin();

  //display();

  // predict
  predictions.clear();
  while(it != this->vehicles.end()) {

    int i = it->first;

    vector<Vehicle> preds = it->second.generate_predictions(prev_size);
    predictions[i] = preds;

    it++;
  }

  // get the best trajectory to advance
  vector<Vehicle> trajectory = ego.choose_next_state(predictions);

  //cout << "best_state " << trajectory[1].state << " lane " << trajectory[1].lane << endl;

  ego.realize_next_state(trajectory);
}

void Road::update_ego(Vehicle vehicle) {

  // update s position (Frenet) coordinate only, don't change lane, speed and state
  ego.s = vehicle.s;

  ego.target_speed = 49.5;
  ego.preferred_buffer = 30;
  ego.lanes_available = 3;
}


