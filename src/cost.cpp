/*
 * cost.cpp
 *
 *  Created on: May 6, 2018
 *      Author: nmkekrop
 */

#include "cost.h"
#include "vehicle.h"
#include <functional>
#include <iterator>
#include <vector>
#include <map>
#include <math.h>


//TODO: change weights for cost functions.
const float LANE_CHANGE = 100.;
const float VELOCITY    = 10000.;
const float LANE_NUMBER = 1000.;

/*
Here we have provided two possible suggestions for cost functions, but feel free to use your own!
The weighted cost over all cost functions is computed in calculate_cost. The data from get_helper_data
will be very useful in your implementation of the cost functions below. Please see get_helper_data
for details on how the helper data is computed.
*/
double get_sigmoid(double delta)  {
  return 1.0/(1.0 + exp(-fabs(delta)));
}

double lane_change_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions, map<string, float> & data) {
  /*
  Penalize for lane change
  */
  Vehicle trajectory_last = trajectory[1];
  double intended_lane;
  double current_lane = vehicle.lane;

  if (trajectory_last.state.compare("PLCL") == 0) {
      intended_lane = trajectory_last.lane + 1;
  } else if (trajectory_last.state.compare("PLCR") == 0) {
      intended_lane = trajectory_last.lane - 1;
  } else {
      intended_lane = current_lane; // give advantage to LCL/R
  }
  //double intended_lane = trajectory[1].lane;

  double delta = (double)(current_lane - intended_lane);

  // use sigmoid to calculate the cost
  double cost = get_sigmoid(delta);

  //cout << "current_lane " << current_lane << " intended_lane " << intended_lane << " LANE cost " << cost  << " state " << trajectory[1].state << endl;
  return cost;
}

double lane_number_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions, map<string, float> & data) {
  /*
  Middle lane is preferred - lowest cost. Penalize for using not preferred lane in "KL' state
  */
  double intended_lane    = trajectory[1].lane;
  double preferred_lane  = 1;
  string state = trajectory[1].state;
  double cost = 0;

  if(state == "KL") {
    cost = fabs(preferred_lane - intended_lane);
  }

  return cost;
}

double velocity_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions, map<string, float> & data) {
    /*
     * Penalize for low speed
    */

    double intended_speed  = trajectory[1].v;

    double delta = (vehicle.target_speed - intended_speed)/vehicle.target_speed;

    // use sigmoid to calculate the cost
    double cost = get_sigmoid(delta);

    //cout << "intended_speed " << intended_speed << " SPEED cost " << cost  << " state " << trajectory[1].state << endl;
    return cost;
}


float inefficiency_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions, map<string, float> & data) {
  /*
  Cost becomes higher for trajectories with intended lane and final lane that have traffic slower than vehicle's target speed.
  You can use the lane_speed(const map<int, vector<Vehicle>> & predictions, int lane) function to determine the speed
  for a lane. This function is very similar to what you have already implemented in the "Implement a Second Cost Function in C++" quiz.
  */
  float cost;
  float proposed_speed_intended = lane_speed(predictions, data["intended_lane"]); // in mph
  //If no vehicle is in the proposed lane, we can travel at target speed.
  if (proposed_speed_intended < 0) {
      proposed_speed_intended = vehicle.target_speed;
  }
/*
  float proposed_speed_final = lane_speed(predictions, data["final_lane"]);
  if (proposed_speed_final < 0) {
      proposed_speed_final = vehicle.target_speed;
  }

  float cost = (2.0*vehicle.target_speed - proposed_speed_intended - proposed_speed_final)/vehicle.target_speed;
*/
  float delta = (vehicle.target_speed - proposed_speed_intended)/vehicle.target_speed;

  cost = 1./(1. + exp(-abs(delta)));

  //cout << vehicle.target_speed << " int_speed " << proposed_speed_intended << " speed cost " << cost  << " state " << trajectory[1].state << endl;
  return cost;
}

float lane_speed(const map<int, vector<Vehicle>> & predictions, int lane) {
  /*
  All non ego vehicles in a lane have the same speed, so to get the speed limit for a lane,
  we can just find one vehicle in that lane.
  */
  for (map<int, vector<Vehicle>>::const_iterator it = predictions.begin(); it != predictions.end(); ++it) {
      int key = it->first;
      Vehicle vehicle = it->second[0];
      if (vehicle.lane == lane && key != -1) {
          return vehicle.v;
      }
  }
  //Found no vehicle in the lane
  return -1.0;
}

double calculate_cost(const Vehicle & vehicle, const map<int, vector<Vehicle>> & predictions, const vector<Vehicle> & trajectory) {
  /*
  Sum weighted cost functions to get total cost for trajectory.
  */
  map<string, float> trajectory_data = get_helper_data(vehicle, trajectory, predictions);
  double cost = 0.0;

  //Add additional cost functions here.
  vector< function<double(const Vehicle & , const vector<Vehicle> &, const map<int, vector<Vehicle>> &, map<string,
      float> &)>> cf_list = {lane_change_cost, lane_number_cost, velocity_cost};

  vector<double> weight_list = {LANE_CHANGE, LANE_NUMBER, VELOCITY};

  for (int i = 0; i < cf_list.size(); i++) {
      double new_cost = weight_list[i]*cf_list[i](vehicle, trajectory, predictions, trajectory_data);
      cost += new_cost;
  }

  return cost;
}

map<string, float> get_helper_data(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions) {
  /*
  Generate helper data to use in cost functions:
  indended_lane: +/- 1 from the current lane if the vehicle is planning or executing a lane change.
  final_lane: The lane of the vehicle at the end of the trajectory. The lane is unchanged for KL and PLCL/PLCR trajectories.
  distance_to_goal: The s distance of the vehicle to the goal.

  Note that indended_lane and final_lane are both included to help differentiate between planning and executing
  a lane change in the cost functions.
  */
  map<string, float> trajectory_data;
  Vehicle trajectory_last = trajectory[1];
  float intended_lane;

  if (trajectory_last.state.compare("LCL") == 0) {
      intended_lane = trajectory_last.lane + 1;
  } else if (trajectory_last.state.compare("LCR") == 0) {
      intended_lane = trajectory_last.lane - 1;
  } else {
      intended_lane = trajectory_last.lane;
  }

  //float distance = vehicle.s - trajectory_last.s;
  float final_lane = trajectory_last.lane;

  trajectory_data["intended_lane"] = intended_lane;
  trajectory_data["final_lane"] = final_lane;
  //trajectory_data["distance"] = distance;
  return trajectory_data;
}



