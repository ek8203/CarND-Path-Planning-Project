/*
 * vehicle.h
 *
 *  Created on: May 6, 2018
 *      Author: nmkekrop
 */

#ifndef VEHICLE_H_
#define VEHICLE_H_

#include <iostream>
#include <random>
#include <vector>
#include <map>
#include <string>

using namespace std;

class Vehicle {
public:

  map<string, int> lane_direction = {{"PLCL", -1}, {"LCL", -1}, {"LCR", 1}, {"PLCR", 1}};

  struct collider{

    bool collision ; // is there a collision?
    int  time; // time collision happens

  };

  //int L = 1;

  double preferred_buffer = 30;//6; // impacts "keep lane" behavior.

  int lane = 1;

  double s;

  double v;
  double measured_v;

  double a;

  double target_speed = 49.5; // in m/s

  int lanes_available = 3;

  double max_acceleration = 5.0;

  int goal_lane = 1;

  string state = "CS";

  int id;

  bool too_close = false;

  /**
  * Constructor
  */
  Vehicle();
  Vehicle(int id, int lane, float s, float v, float a, string state="CS");

  /**
  * Destructor
  */
  virtual ~Vehicle();

  vector<Vehicle> choose_next_state(map<int, vector<Vehicle>> predictions);

  vector<string> successor_states();

  vector<Vehicle> generate_trajectory(string state, map<int, vector<Vehicle>> predictions);

  vector<float> get_kinematics(map<int, vector<Vehicle>> predictions, int lane);

  vector<Vehicle> constant_speed_trajectory();

  vector<Vehicle> keep_lane_trajectory(map<int, vector<Vehicle>> predictions);

  vector<Vehicle> lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions);

  vector<Vehicle> prep_lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions);

  void increment(int dt);

  float position_at(int t);
  double position_at(int prev_size, double dt);

  bool get_vehicle_behind(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle);

  bool get_vehicle_ahead(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle);

  vector<Vehicle> generate_predictions(int horizon=2);

  void realize_next_state(vector<Vehicle> trajectory);

  void configure(vector<double> road_data);

  void display();

};

#endif /* VEHICLE_H_ */
