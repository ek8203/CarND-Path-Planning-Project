/*
 * vehicle.cpp
 *
 *  Created on: May 6, 2018
 *      Author: nmkekrop
 */

#include <algorithm>
#include <iostream>
#include "vehicle.h"
#include <cmath>
#include <map>
#include <string>
#include <iterator>
#include "cost.h"

/**
 * Initializes Vehicle
 */

Vehicle::Vehicle(){
  max_acceleration = 0.224;
  target_speed = 49.5;
  lanes_available = 3;
  goal_lane = 1;
}

Vehicle::Vehicle(int id, int lane, float s, float v, float a, string state) {

  this->id = id;
  this->lane = lane;
  this->s = s;
  this->v = v;
  this->a = a;
  this->state = state;

  max_acceleration = 0.224;
  target_speed = 49.5;
  lanes_available = 3;
  goal_lane = lane;
}

Vehicle::~Vehicle() {}


vector<Vehicle> Vehicle::choose_next_state(map<int, vector<Vehicle>> predictions) {
    /*

    ***Here you can implement the transition_function code from the Behavior Planning Pseudocode
    classroom concept.***

    INPUT: A predictions map. This is a map using vehicle id as keys with predicted
        vehicle trajectories as values. A trajectory is a vector of Vehicle objects. The first
        item in the trajectory represents the vehicle at the current timestep. The second item in
        the trajectory represents the vehicle one timestep in the future.
    OUTPUT: The the best (lowest cost) trajectory for the ego vehicle corresponding to the next ego vehicle state.

    Functions that will be useful:
    1. successor_states() - Uses the current state to return a vector of possible successor states for the finite
       state machine.
    2. generate_trajectory(string state, map<int, vector<Vehicle>> predictions) - Returns a vector of Vehicle objects
       representing a vehicle trajectory, given a state and predictions. Note that trajectory vectors
       might have size 0 if no possible trajectory exists for the state.
    3. calculate_cost(Vehicle vehicle, map<int, vector<Vehicle>> predictions, vector<Vehicle> trajectory) - Included from
       cost.cpp, computes the cost for a trajectory.
    */

  //TODO: Your solution here.
  // List of next states
  vector<string> states = successor_states();

  float cost;
  vector<float> costs;
  vector<string> final_states;
  vector<vector<Vehicle>> final_trajectories;

  // For each state get a trajectory and calculate cost
  for (vector<string>::iterator it = states.begin(); it != states.end(); ++it) {

    vector<Vehicle> trajectory = generate_trajectory(*it, predictions);

    if (trajectory.size() != 0) {

      cost = calculate_cost(*this, predictions, trajectory);
      costs.push_back(cost);

      cout << "lane " << trajectory[1].lane << " state " << trajectory[1].state << " cost " << cost << endl;

      final_trajectories.push_back(trajectory);
    }
  }

  float min_cost = costs[0];
  int best_idx = 0;
  for(int i = 0; i < final_trajectories.size(); i++)  {
    if(costs[i] < min_cost) {
      min_cost = costs[i];
      best_idx = i;
    }
  }

  cout << "best lane " << final_trajectories[best_idx][1].lane << " state " << final_trajectories[best_idx][1].state << " cost " << min_cost << endl;

  return final_trajectories[best_idx];
}

vector<string> Vehicle::successor_states() {

  /*
  Provides the possible next states given the current state for the FSM
  discussed in the course, with the exception that lane changes happen
  instantaneously, so LCL and LCR can only transition back to KL.
  */
  // array of successor states
  vector<string> states;

  // To KL from any state
  states.push_back("KL");

  // Current state
  string state = this->state;

  // KL state
  if(state.compare("KL") == 0) {
    if (lane != 0)
      states.push_back("PLCL");
    if (lane != lanes_available - 1)
      states.push_back("PLCR");
  }
  // PLCL
  else if (state.compare("PLCL") == 0) {
    // left
    if (lane != 0) {
      states.push_back("PLCL");
      states.push_back("LCL");
    }
  }
  // PLCR
  else if (state.compare("PLCR") == 0) {
    // right
    if (lane != lanes_available - 1) {
      states.push_back("PLCR");
      states.push_back("LCR");
    }
  }
  //If state is "LCL" or "LCR", then just return "KL"

  return states;
}

vector<Vehicle> Vehicle::generate_trajectory(string state, map<int, vector<Vehicle>> predictions) {
  /*
  Given a possible next state, generate the appropriate trajectory to realize the next state.
  */
  vector<Vehicle> trajectory;

  if (state.compare("CS") == 0) {
    trajectory = constant_speed_trajectory();
  } else if (state.compare("KL") == 0) {
    trajectory = keep_lane_trajectory(predictions);
  } else if (state.compare("LCL") == 0 || state.compare("LCR") == 0) {
    trajectory = lane_change_trajectory(state, predictions);
  } else if (state.compare("PLCL") == 0 || state.compare("PLCR") == 0) {
    trajectory = prep_lane_change_trajectory(state, predictions);
  }

  return trajectory;
}

vector<float> Vehicle::get_kinematics(map<int, vector<Vehicle>> predictions, int lane) {
  /*
  Gets next timestep kinematics (position, velocity, acceleration)
  for a given lane. Tries to choose the maximum velocity and acceleration,
  given other vehicle positions and accel/velocity constraints.
  */
  float max_velocity_accel_limit = this->max_acceleration + this->v;
  float new_position;
  float new_velocity;
  float new_accel;
  Vehicle vehicle_ahead;
  Vehicle vehicle_behind;

  if (get_vehicle_ahead(predictions, lane, vehicle_ahead)) {

    if(this->lane == lane) {
      //cout << "vehicle " << vehicle_ahead.id << " ahead " << vehicle_ahead.s - this->s << endl;
      cout << "vehicle " << vehicle_ahead.id << " ahead " << vehicle_ahead.v*2.24 << endl;
    }

    this->too_close = true;
    new_velocity = vehicle_ahead.v * 2.24; // cope to speed ahead
  }
  else {
    if(this->too_close && (this->lane == lane)) {
      cout << "not too close!!!" << endl;
      this->too_close = false;
    }

    // if no vehicles ahead, drive at maximum speed
    new_velocity = this->target_speed;
  }

  // TODO: correct to real values - not in use for now
  new_accel = new_velocity - this->v; //Equation: (v_1 - v_0)/t = acceleration
  new_position = this->s + new_velocity + new_accel / 2.0;

  return{new_position, new_velocity, new_accel};
}

vector<Vehicle> Vehicle::constant_speed_trajectory() {
  /*
  Generate a constant speed trajectory.
  */

  float next_pos = position_at(0, 0.02);

  vector<Vehicle> trajectory = {Vehicle(this->id, this->lane, this->s, this->v, this->a, this->state),
                                  Vehicle(this->id, this->lane, next_pos, this->v, 0, this->state)};
  return trajectory;
}

vector<Vehicle> Vehicle::keep_lane_trajectory(map<int, vector<Vehicle>> predictions) {
  /*
  Generate a keep lane trajectory.
  */
  vector<Vehicle> trajectory = {Vehicle(this->id, this->lane, this->s, this->v, this->a, state)};

  vector<float> kinematics = get_kinematics(predictions, this->lane);
  float new_s = kinematics[0];
  float new_v = kinematics[1];
  float new_a = kinematics[2];

  // KL speed is used as the lane speed
  this->max_lane_v = new_v;

  trajectory.push_back(Vehicle(this->id, this->lane, new_s, new_v, new_a, "KL"));
  return trajectory;
}

vector<Vehicle> Vehicle::prep_lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions) {
  /*
  Generate a trajectory preparing for a lane change.
  */
  float new_s;
  float new_v;
  float new_a;
  Vehicle vehicle_behind;

  int new_lane = this->lane + lane_direction[state];

  vector<Vehicle> trajectory = {Vehicle(this->id, this->lane, this->s, this->v, this->a, this->state)};

  vector<float> curr_lane_new_kinematics = get_kinematics(predictions, this->lane);

  bool is_new_lane_busy = get_vehicle_behind(predictions, new_lane, vehicle_behind);

  if (is_new_lane_busy && vehicle_behind.v*2.24 > this->v) {

    //Keep speed of current lane so as not to collide with car behind.
    new_s = curr_lane_new_kinematics[0];
    new_v = curr_lane_new_kinematics[1];
    new_a = curr_lane_new_kinematics[2];

    cout << "vehicle " << vehicle_behind.id << " behind " << " lane " << vehicle_behind.lane <<" speed " << vehicle_behind.v*2.24 << endl;
  } else {

    vector<float> best_kinematics;
    vector<float> next_lane_new_kinematics = get_kinematics(predictions, new_lane);

    //Choose kinematics with highest velocity.
    if (next_lane_new_kinematics[1] > curr_lane_new_kinematics[1]) {
        best_kinematics = next_lane_new_kinematics;
    } else {
        best_kinematics = curr_lane_new_kinematics;
    }

    new_s = best_kinematics[0];
    new_v = best_kinematics[1];
    new_a = best_kinematics[2];
  }

  // keep current lane
  trajectory.push_back(Vehicle(this->id, this->lane, new_s, new_v, new_a, state));

  return trajectory;
}

vector<Vehicle> Vehicle::lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions) {
  /*
  Generate a lane change trajectory.
  */
  int new_lane = this->lane + lane_direction[state];
  vector<Vehicle> trajectory;
  Vehicle next_lane_vehicle;

  //Check if a lane change is possible (check if another vehicle occupies that spot).
  for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {

    next_lane_vehicle = it->second[0];

    double range = fabs(next_lane_vehicle.s - this->s);

    if (range < 30.0 && next_lane_vehicle.lane == new_lane) {
      //If lane change is not possible, return empty trajectory.
      cout << "can't move to " << state << " id " << next_lane_vehicle.id << " in range " << range << endl;
      return trajectory;
    }
  }

  trajectory.push_back(Vehicle(this->id, this->lane, this->s, this->v, this->a, this->state));

  vector<float> kinematics = get_kinematics(predictions, new_lane);

  cout << "intend change to lane " << new_lane << endl;

  // change the lane
  trajectory.push_back(Vehicle(this->id, new_lane, kinematics[0], kinematics[1], kinematics[2], state));

  return trajectory;
}

double Vehicle::position_at(int prev_size, double dt) {

  return this->s + (double)prev_size * dt * this->v;
}

bool Vehicle::get_vehicle_behind(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle) {
  /*
  Returns a true if a vehicle is found behind the current vehicle, false otherwise. The passed reference
  rVehicle is updated if a vehicle is found.
  */
  double min_range = this->preferred_buffer;
  bool found_vehicle = false;
  Vehicle temp_vehicle;

  for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {

    temp_vehicle = it->second[0];

    double range = fabs(temp_vehicle.s - this->s);

    if (temp_vehicle.lane == lane && temp_vehicle.s < this->s && range < min_range) {
      min_range = range;
      rVehicle = temp_vehicle;
      found_vehicle = true;
    }
  }

  return found_vehicle;
}

bool Vehicle::get_vehicle_ahead(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle) {
  /*
  Returns a true if a vehicle is found ahead of the current vehicle, false otherwise. The passed reference
  rVehicle is updated if a vehicle is found.
  */
  double min_range = this->preferred_buffer;
  bool found_vehicle = false;
  Vehicle temp_vehicle;

  for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {

    temp_vehicle = it->second[0];

    double range = fabs(temp_vehicle.s - this->s);

    if (temp_vehicle.lane == lane && temp_vehicle.s > this->s && range < min_range) {
      min_range = range;
      rVehicle = temp_vehicle;
      found_vehicle = true;
    }
  }

  return found_vehicle;
}

vector<Vehicle> Vehicle::generate_predictions(int prev_size) {
  /*
   * Generates predictions for non-ego vehicles to be used
   * in trajectory generation for the ego vehicle.
   */
	vector<Vehicle> predictions;

  double next_s = position_at(prev_size, 0.02);

  predictions.push_back(Vehicle(this->id, this->lane, next_s, this->v, 0));

	return predictions;
}

void Vehicle::realize_next_state(vector<Vehicle> trajectory) {
  /*
  Sets state and kinematics for ego vehicle using the last state of the trajectory.
  */
  Vehicle next_state = trajectory[1];
  this->state = next_state.state;
  this->lane = next_state.lane;
  this->s = next_state.s;

  //double next_v = next_state.v;
  double next_v = this->max_lane_v;

  // control the speed to avoid jerk
  if(this->v > next_v)  {
    if(this->too_close)
      this->v -= max_acceleration * 2.5; // faster deceleration to avoid collision
    else
      this->v -= max_acceleration;
  }
  else if(!this->too_close){
    this->v += max_acceleration;
  }

  cout << "next_v " << next_v << " ego_v " << this->v << " state " << this->state << endl;

  this->a = next_state.a;
}

void Vehicle::display()  {
  cout << this->id << " speed " << this->v << " lane " << this->lane;
  cout << " state " << this->state << endl;
}

