
### Sefl-Driving Car Nanodegree Program. Term 3
<img style="float: left;" src="https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg">

## Project 11: Path Planning Project

### Overview
---
The goal of this project is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. The project simulator provides the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

### Project directory content:

* [README.md](README.md) - This file.
* [src/](src/) folder:
   - project C++ source code files
   - [CMakeLists.txt](src/CMakeLists.txt) - `cmake` input file to generate the project `Makefile`. It was moved to the `src` folder and adopted to generate an `Eclipse` project - thanks to [Eclipse IDE profile](https://github.com/udacity/CarND-Extended-Kalman-Filter-Project/tree/master/ide_profiles/Eclipse) repo for provided instructions.
* [data/](data/) folder:
    - [highway_map.txt](data/highway_map.txt) - Contains a map of the highway's waypoints. Each waypoint in the list contains [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop. The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

### Project environment


The project code was built and executed on a VMware Linux guest.

### Project build and code run instructions

1. Make a build directory: `mkdir build && cd build`
2. Compile the code: `cmake ../src/ && make` (Note that `CMakeList.txt` file is located in folder `src/`)
3. Run the code: `./path_planning`

### The data provided from the Simulator

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along the path has processed since last time.

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates.

### Details
---
* The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

* There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

### Reflection

#### General
The provided solution takes credit of 2 class lessons:

* [Implement Behavior Planner in C++](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/27800789-bc8e-4adc-afe0-ec781e82ceae/lessons/56274ea4-277d-4d1e-bd95-ce5afbad64fd/concepts/2c4a8fdd-4072-425f-b5ae-95849d5fd4d8) quiz
* [Project Walkthrogh and Q&A](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/27800789-bc8e-4adc-afe0-ec781e82ceae/lessons/23add5c6-7004-47ad-b169-49a5d7b1c1cb/concepts/3bdfeb8c-8dd6-49a7-9d08-beff6703792d) video

Even so the solution is simplified, it is providing a smooth driving, decent speed and lane change maneuvers in the simulated highway traffic. Beside, it covers the three basic Path Planning steps: 
1. Prediction
2. Behavior Planning
3. Trajectory generation

The main program flow is following:
* Update the road traffic with `sensor_fusion` data reveived from the simulator using `update_traffic()` method of the `Road` class
* Update the `ego` vehicle with the `car` data from the simulator using `update_ego()` class method
* Execute the `advance_ego()` class method to go trough the path planning steps: `prediction`, `behavior planning` and generation of the best `predicted trajectory`
* Get the best predicted `speed` and `lane` number using the `get_ego()` class method
* Calculate the next vehicle trajectory waypoints using [spline](http://kluge.in-chemnitz.de/opensource/spline/) function as described the [Walkthrogh](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/27800789-bc8e-4adc-afe0-ec781e82ceae/lessons/23add5c6-7004-47ad-b169-49a5d7b1c1cb/concepts/3bdfeb8c-8dd6-49a7-9d08-beff6703792d) video (this step is not described here since it is clear explained in the video)

In order to avoid jerk acceleration the speed is controled in steps of +/-0.224 mph unless there is a risk of collision. 

#### Prediction

The prediction is calculated for every non-ego vehicle on the road. The calculation is using the method presented in the [Walkthrogh](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/27800789-bc8e-4adc-afe0-ec781e82ceae/lessons/23add5c6-7004-47ad-b169-49a5d7b1c1cb/concepts/3bdfeb8c-8dd6-49a7-9d08-beff6703792d) video. It is calculating the `s` position of a vehicle in the future based on the vehicle velocity and the size of the previous `(x, y)` position points list given to the simulator:    
```cplusplus
double Vehicle::position_at(int prev_size, double dt) {
  return this->s + (double)prev_size * dt * this->v;
}
```
The parameter `dt` is the simulator data update rate, 0.02 sec. The velocity `v` is the magnitude of the velocity vector calculated from `vx` and `vy` sensor fusion data:
```cplusplus
double v = sqrt(vx*vx + vy*vy);
```
The resulting prediction vector is passed to the Behavior Planner for processing.

#### Behavior Planning

The Behavior Planner is implemented as a Finite State Machine that sets the ego vehicle to one of following states:

* "KL" - Keep Lane
* "PLCL" / "PLCR" - Prepare Lane Change Left / Prepare Lane Change Right
* "LCL" / "LCR"- Lane Change Left / Lane Change Right

First, a list of successor states is build for a current ego vehicle state using `successor_states()` method of the Vehicle class. Next, given the calculated prediction vector, a method `generate_trajectory()` is called to build ego vehicle trajectories for every successor state. Then the cost of every trajectory is calculated by calling `calculate_cost()` function. The trajectory with the minimum cost is chosen as a final trajectory of the ego vehicle and is used to calculate the next vehicle trajectory waypoints using the `spline` function


#### Trajectory cost

The cost of the ego vehicle potential trajectories is calculated using following cost functions:

* `lane_change_cost()` - calculates a cost of changing lanes since driving with less lane changes is safer and more comfortable
* `lane_number_cost()` - gives minimum cost for driving in the prefered lane. The middle lane was chosen as the prefered lane in this project
* `velocity_cost()` - panalizes for driving low speed.

The cost functions are using `sigmoid()` function to return a value in range from 0 to 1. Then every cost value is multiplied by its predefined `weight`. Changing the values of `wwight` is adjusting the driving behavior of the ego vehicle.   


### Issues and points to improve

The provided solution is not perfect but was able to drive the ego vehicle one full lap in most of the tests without collisions at decent speed. The main issue of this implementation is low accuracy because it is using the Fernet coordinates only to calculate the predictions. The prediction model is too simplified. Vehicle acceleration values are not calculated and not used in the model. Some logic is still used for choosing a next state of the ego vehicle. Idially all decision making should be done using the cost functions. Also more cost functions could be used to adjust the driving behavior.

