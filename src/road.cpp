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
    // the lanes are equal
    this->lane_speeds = {speed_limit, speed_limit, speed_limit};
    this->speed_limit = speed_limit;

    // redundant
    this->density = 0;
    this->camera_center = this->update_width/2;

}

Road::Road() {
  this->ego = Vehicle(ego_key, 1, 0, 0, 0);
  ego.state = "KL";
  ego.too_close = false;
}

Road::~Road() {}

Vehicle Road::get_ego() {

//  return this->vehicles.find(this->ego_key)->second;
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


void Road::update_traffic(map<int, Vehicle> & traffic)
{
  // remove vehicles not in range of sensor
  for(map<int, Vehicle>::iterator it = vehicles.begin(); it != vehicles.end(); ++it) {

    int v_id = it->first; // get key

    if(v_id != ego_key) {
      map<int, Vehicle>::iterator t_it = traffic.find(v_id);
      // not found if not in range
      if(t_it == traffic.end()) {
        // remove existing vehicle from road
        //vehicles.erase(v_id);
        //cout << "remove: " << v_id << " size: " << vehicles.size() << endl;
        //break;
      }
      // otherwise update existing vehicle
      else if(v_id == t_it->first) {
        //cout << "update " << v_id << endl;
        Vehicle v = vehicles[v_id];
        Vehicle t = t_it->second;

        double a = t.v - v.v;
        v.id = t.id;
        v.lane = t.lane;
        v.s = t.s;
        v.v = t.v;
        v.a = a;
      }
    }
  }

  // Add if new
  for(map<int, Vehicle>::iterator it = traffic.begin(); it != traffic.end(); ++it) {

    int t_id = it->first; // get key

    Vehicle vehicle = it->second; // get vehicle

    // add if not found
    if(vehicles.find(t_id) == vehicles.end()) {
      vehicle.state = "CS";
      //vehicle.id = t_id;
      vehicles.insert(std::pair<int,Vehicle>(t_id, vehicle));
      //cout << "add: " << it->first << " lane: " << vehicle.lane << " size: " << vehicles.size() << endl;
    }
  }
}

void Road::populate_traffic() {

	int start_s = max(this->camera_center - (this->update_width/2), 0);
	for (int l = 0; l < this->num_lanes; l++)
	{
		int lane_speed = this->lane_speeds[l];
		bool vehicle_just_added = false;
		for(int s = start_s; s < start_s+this->update_width; s++)
		{

			if(vehicle_just_added)
			{
				vehicle_just_added = false;
			}
			if(((double) rand() / (RAND_MAX)) < this->density)
			{

				Vehicle vehicle = Vehicle(0, l,s,lane_speed,0);
				vehicle.state = "CS";
				this->vehicles_added += 1;
				this->vehicles.insert(std::pair<int,Vehicle>(vehicles_added,vehicle));
				vehicle_just_added = true;
			}
		}
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

void Road::advance() {

  map<int ,vector<Vehicle> > predictions;
  map<int, Vehicle>::iterator it = this->vehicles.begin();

  // predict
  while(it != this->vehicles.end()) {

    int v_id = it->first;

    vector<Vehicle> preds = it->second.generate_predictions();
    predictions[v_id] = preds;
    it++;
  }

  it = this->vehicles.begin();
  while(it != this->vehicles.end()) {

    int v_id = it->first;

    if(v_id == ego_key) {
      vector<Vehicle> trajectory = it->second.choose_next_state(predictions);

      it->second.realize_next_state(trajectory);
    }
    else {
      it->second.increment(1);
    }

    it++;
  }
}

void Road::update_ego(double x, double y, double s, double d, double yaw, double speed) {

  map<int, Vehicle>::iterator it;
  int lane_num = (int)(d/4);
  //double vx = speed * cos(yaw);
  //double vy = speed * sin(yaw);

  // find ego
  it = vehicles.find(ego_key);

  // if first time - not found
  if(it == vehicles.end())  {
    double SPEED_LIMIT = 49.5;
    double MAX_ACCEL = .224;
    vector<double> ego_config = {SPEED_LIMIT, MAX_ACCEL};
    add_ego(lane_num, s, ego_config);
    //cout << "add ego" << endl;
  }
  else {
    // update ego
    Vehicle ego = it->second;

    // new acceleration
    double a = speed - ego.v;
    if(a < -0.224) a = -0.224;
    else if(a > 0.224) a = 0.224;
    ego.a = a;

    ego.s = s;

    if(ego.too_close)
    {
      ego.v -= 0.224;
    }
    else if(ego.v < 49.5) {
      ego.v += 0.224;
    }

    //cout << "upadte ego" << endl;
    vehicles[ego_key] = ego;
  }
}

void Road::update_ego(Vehicle vehicle) {

  // update s position coordinate only, don't change lane, speed and state
  ego.s           = vehicle.s;
  ego.measured_v  = vehicle.v;

  ego.target_speed = 49.5;
  ego.preferred_buffer = 30;
  ego.lanes_available = 3;

  //cout << "ego lane " << ego.lane << " state " << ego.state << " speed " << ego.v << endl;
}

void Road::add_ego(int lane_num, int s, vector<double> config_data) {

	map<int, Vehicle>::iterator it = this->vehicles.begin();
  while(it != this->vehicles.end())
  {
    int v_id = it->first;
      Vehicle v = it->second;
      if(v.lane == lane_num && v.s == s)
      {
        this->vehicles.erase(v_id);
      }
      it++;
  }
  Vehicle ego = Vehicle(ego_key, lane_num, s, 0, 0);
  ego.configure(config_data);
  ego.state = "KL";
  ego.too_close = false;
  this->vehicles.insert(std::pair<int,Vehicle>(ego_key,ego));
}

void Road::display(int timestep) {

    Vehicle ego = this->vehicles.find(this->ego_key)->second;
    int s = ego.s;
    string state = ego.state;

    this->camera_center = max(s, this->update_width/2);
    int s_min = max(this->camera_center - this->update_width/2, 0);
    int s_max = s_min + this->update_width;

    vector<vector<string> > road;

    for(int i = 0; i < this->update_width; i++)
    {
        vector<string> road_lane;
        for(int ln = 0; ln < this->num_lanes; ln++)
        {
            road_lane.push_back("     ");
        }
        road.push_back(road_lane);

    }

    map<int, Vehicle>::iterator it = this->vehicles.begin();
    while(it != this->vehicles.end())
    {

        int v_id = it->first;
        Vehicle v = it->second;

        if(s_min <= v.s && v.s < s_max)
        {
            string marker = "";
            if(v_id == this->ego_key)
            {
                marker = this->ego_rep;
            }
            else
            {

                stringstream oss;
                stringstream buffer;
                buffer << " ";
                oss << v_id;
                for(int buffer_i = oss.str().length(); buffer_i < 3; buffer_i++)
                {
                    buffer << "0";

                }
                buffer << oss.str() << " ";
                marker = buffer.str();
            }
            road[int(v.s - s_min)][int(v.lane)] = marker;
        }
        it++;
    }
    ostringstream oss;
    oss << "+Meters ======================+ step: " << timestep << endl;
    int i = s_min;
    for(int lj = 0; lj < road.size(); lj++)
    {
        if(i%20 ==0)
        {
            stringstream buffer;
            stringstream dis;
            dis << i;
            for(int buffer_i = dis.str().length(); buffer_i < 3; buffer_i++)
            {
                 buffer << "0";
            }

            oss << buffer.str() << dis.str() << " - ";
        }
        else
        {
            oss << "      ";
        }
        i++;
        for(int li = 0; li < road[0].size(); li++)
        {
            oss << "|" << road[lj][li];
        }
        oss << "|";
        oss << "\n";
    }

    cout << oss.str();

}



