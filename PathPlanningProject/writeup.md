# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   

### Goals
this purpose of this project is to provide future trajectory points (no control module, the car will directly move to the provided way points)

#### the given data
1. car's location
2. sensor fusion data (the status of other cars)
3. high way map

#### the requirements of the car behavior:
1. move as close as 50 MPH 
2. don't hit other cars
3. run on the high way lanes (there are 3 lanes)
4. be able to switch from one lane to another if necessary (the front car is too slow and the target lane is available)
5. less jerk, e.g., no sudden acceleration on the lateral or forward.
(total acceleration < 10m/s^2, jerk < 10m/s^2)


#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.


#### implementation
we implemented this one step by one, more and more powerful and complex.
(I implemented step by step, but only the final codes left in the repo)

1. make the car move in a straight line

see the codes in function strategy 1

2. keep turing a circle startegy
see the code in function strategy 2
we will manually calc the car orientation angle from the previous points and calc the car pos and angle from previous points, then also keep the straight lane direction

3. keep in the lane strategy
see the coe in function strategy 3
just keep the s increasing and d un-changed

4. need to try to make the future points smooth that in case we meet a sharp turn, we can still meet the jerk requirement
see the code in function strtegy 4.
spline function used to help for the interpolation
use 80 points for the future points number.

I tried several different path point numbers,
use 50, which induce speed = 0 error, as the simulator will consume more than 50 points between 2 update from the server.
use 100, in condition there's a sharp turn, such long range prediction is not reliable.
800 in my test is fine.




5. use sensor fusion and detect if there's a slow car in the front of ego
then decide to slow down the car to avoid collision
see code in function strategy 5.
I introduced a safe_distance concept, which depends on the speed, as 1s * speed.
e.g., a driver will alays have 1s response time to take action before collision.


6. use gradually update the ref velocity to avoid jerk.
also see code in function strategy 5.
when the front path is safe, speed up by 0.225 m/s
when the front path is not safe, slow down by 0.225 m/s

7. switch lane strategy
see the help function is_target_lane_safe() to check if it's safe to switch to the neighbor lane.
switch lane only at condition that the target lane is safe, and speed is loweer than 48./
the speed limitation is that:
when we switch a lane, even our ref velocity is under 50m/s,
that the predicted lane path points will not be a flat straight lane in the car-coordinate,
thus will induce higher actual speed. we need to give more safe space for the speed.




## Basic Build Instructions



#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

