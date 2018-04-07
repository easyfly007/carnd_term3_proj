#include <vector>
#include <math.h>
#include <iostream>

#include <assert.h>
#include "spline.h"

using namespace std;

extern double deg2rad(double x);

constexpr double pi() { return M_PI; }

extern vector<double> getFrenet(double x, double y, double theta, 
	const vector<double> &maps_x, const vector<double> &maps_y);

extern vector<double> getXY(double s, double d, const vector<double> &maps_s, 
	const vector<double> &maps_x, const vector<double> &maps_y);

const int max_path_points = 80;

int getLane(double d)
{
	return 1;
	if (0.0 <= d && d <= 4.0)
		return 0;
	if (4.0 <= d && d <= 8.0)
		return 1;
	return 2;
}

void path_plan_strategy1(vector<double> &next_x_vals, vector<double> &next_y_vals, 
	double car_yaw, double car_x, double car_y, double car_v)
{
	// input: car_yaw
	// output: next_x_vals, next_y_vals

	// make the car move in a straight line
	double dist_inc = 0.225;
	for (int i = 1; i < max_path_points; i ++)
	{
		next_x_vals.push_back(car_x + (dist_inc * i) * cos(deg2rad(car_yaw)) );
		next_y_vals.push_back(car_y + (dist_inc * i) * sin(deg2rad(car_yaw)));
	}
	return;
}


void path_plan_strategy2(vector<double> &next_x_vals, vector<double> &next_y_vals, 
	double car_yaw, double car_x, double car_y, 
	vector<double> &previous_path_x, vector<double> &previous_path_y )
{
	// keep the car turing around in a circle 
	double pos_x, pos_y;
	double angle;
	int path_size = previous_path_x.size();
	assert(previous_path_x.size() == previous_path_y.size());
	double dist_inc = 0.5;

	for (int i = 0; i < path_size; i ++)
	{
		next_x_vals.push_back(previous_path_x[i]);
		next_y_vals.push_back(previous_path_y[i]);
	}

	if (path_size == 0)
	{
		pos_x = car_x;
		pos_y = car_y;
		angle = deg2rad(car_yaw);
	}
	else
	{
		pos_x = previous_path_x[path_size - 1];
		pos_y = previous_path_y[path_size - 1];
		double pos_x2 = previous_path_x[path_size - 2];
		double pos_y2 = previous_path_y[path_size - 2];
		angle = atan2(pos_y - pos_y2, pos_x - pos_x2);
	}

	for (int i = 0; i < max_path_points - path_size; i ++)
	{
		next_x_vals.push_back(pos_x + dist_inc * cos(angle + (i + 1) * pi() / 100));
		next_y_vals.push_back(pos_y + dist_inc * sin(angle + (i + 1) * pi() / 100));
		pos_x += dist_inc * cos(angle + (i + 1) * pi() / 100);
		pos_y += dist_inc * sin(angle + (i + 1) * pi() / 100);
	}
}


void path_plan_strategy3(
	vector<double> &next_x_vals, vector<double> &next_y_vals, 
	double car_yaw, double car_x, double car_y, 
	vector<double> &previous_path_x, vector<double> &previous_path_y,
	vector<double> &map_waypoints_x,
	vector<double> &map_waypoints_y,
	vector<double> &map_waypoints_s,
	vector<double> &map_waypoints_dx,
	vector<double> &map_waypoints_dy)
{
	double dist_inc = 0.5;
	double pos_x = car_x;
	double pos_y = car_y;
	int path_size = previous_path_x.size();
	double angle = deg2rad(car_yaw);
	if (path_size > 1)
	{
		pos_x = previous_path_x[path_size -1];
		pos_y = previous_path_y[path_size -1];
		double pos_x2 = previous_path_x[path_size -2];
		double pos_y2 = previous_path_y[path_size -2];
		angle = atan2(pos_y - pos_y2, pos_x - pos_x2);
	}
	vector<double> pos_sd = getFrenet(pos_x, pos_y, angle, map_waypoints_x, map_waypoints_y);
	double pos_s = pos_sd[0];
	double pos_d = pos_sd[1];
	for (int i = 0; i < max_path_points; i ++)
	{
		double next_s = pos_s + (i + 1) * dist_inc;
		double next_d = pos_d;
		vector<double> next_xy = getXY(next_s, next_d, map_waypoints_s,map_waypoints_x,map_waypoints_y);
		double next_x = next_xy[0];
		double next_y = next_xy[1];
		next_x_vals.push_back(next_x);
		next_y_vals.push_back(next_y);
	}
}

bool is_target_lane_safe(int target_lane, const vector<vector<double> > & sensor_fusion,
	double car_yaw, double car_s, double car_d, double car_v, int prev_size)
{
	double safe_dist_ahead = car_v * 1.0;
	if (safe_dist_ahead < 30)
		safe_dist_ahead = 30;
	double safe_dist_behind = 30;
	
	for (int i = 0; i < sensor_fusion.size(); i ++)
	{
		double obs_car_id = sensor_fusion[i][0];
		double obs_car_x  = sensor_fusion[i][1];
		double obs_car_y  = sensor_fusion[i][2];
		double obs_car_vx = sensor_fusion[i][3];
		double obs_car_vy = sensor_fusion[i][4];
		double obs_car_s  = sensor_fusion[i][5];
		double obs_car_d  = sensor_fusion[i][6];
		double obs_car_v  = sqrt(obs_car_vx*obs_car_vx + obs_car_vy*obs_car_vy);

		if (obs_car_d > target_lane * 4 && obs_car_d <target_lane * 4 + 4 )
		{
			if (obs_car_v <= car_v && obs_car_s >= car_s && obs_car_s < car_s + safe_dist_ahead)
			{
				cout << "not safe 1, target_lane= " << target_lane << endl;
				return false;
			}
			else if (obs_car_s > car_s)
			{
				double check_car_s = obs_car_s + prev_size * 0.02 * obs_car_v;
				// we will check in a futuer s range that if ego car and checked are will collision
				if (check_car_s > car_s && check_car_s - car_s < 30)
				{
					cout << "not safe 2, target lane = " << target_lane << endl;
					return false;
				}
			}
			else
			{
				// check car behind 
				if (obs_car_s + safe_dist_behind > car_s && obs_car_v > car_v-0.2)
				{
					cout << "not safe 3, target_lane = " << target_lane << endl;
					return false;
				}
			}
		}
	}
	return true;
}


// smooth the line
void path_plan_strategy4(
	vector<double> &next_x_vals, vector<double> &next_y_vals, int current_lane, int target_lane,
	double car_yaw, double car_s, double car_d, double car_x, double car_y, double ref_v,
	vector<double> &previous_path_x, vector<double> &previous_path_y,
	vector<double> &map_waypoints_x,
	vector<double> &map_waypoints_y,
	vector<double> &map_waypoints_s,
	vector<double> &map_waypoints_dx,
	vector<double> &map_waypoints_dy)
{
	// strategy 4, to make the turing more smooth
	vector<double> ptsx, ptsy;
	double ref_x = car_x, ref_y = car_y;
	double ref_yaw = deg2rad(car_yaw);
	int lane = getLane(car_s);
	// velocity limit is 50, we will set the ref vel close to limit, but not exceed it 
	int path_size = previous_path_x.size();
	if (path_size < 2)
	{
		double prev_car_x = car_x - cos(car_yaw);
		double prev_car_y = car_y - sin(car_yaw);
		ptsx.push_back(prev_car_x);
		ptsy.push_back(prev_car_y);
		ptsx.push_back(car_x);
		ptsy.push_back(car_y);
	}
	else
	{
		ref_x = previous_path_x[path_size -1];
		ref_y = previous_path_y[path_size -1];
		double ref_x2 = previous_path_x[path_size -2];
		double ref_y2 = previous_path_y[path_size -2];
		ref_yaw = atan2(ref_y - ref_y2, ref_x - ref_x2);
		ptsx.push_back(ref_x2);
		ptsy.push_back(ref_y2);
		ptsx.push_back(ref_x);
		ptsy.push_back(ref_y);
	}
	double safe_dist = ref_v;
	vector<double> next_wp0 = getXY(car_s + ref_v * 1, 4 * target_lane + 2,
		map_waypoints_s, map_waypoints_x,map_waypoints_y);
	vector<double> next_wp1 = getXY(car_s + ref_v * 2, 4 * target_lane + 2, 
		map_waypoints_s, map_waypoints_x,map_waypoints_y);
	vector<double> next_wp2 = getXY(car_s + ref_v * 3, 4 * target_lane + 2, 
		map_waypoints_s, map_waypoints_x,map_waypoints_y);
	
	// data points for smoothing

	ptsx.push_back(next_wp0[0]);
	ptsx.push_back(next_wp1[0]);
	ptsx.push_back(next_wp2[0]);
	ptsy.push_back(next_wp0[1]);
	ptsy.push_back(next_wp1[1]);
	ptsy.push_back(next_wp2[1]);

	for (int i = 0; i < ptsx.size(); i ++)
	{
		// shift the points from the map coordinate to car coordinate
		// which will make the fitting better
		double shift_x = ptsx[i] - ref_x;
		double shift_y = ptsy[i] - ref_y;
		ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
		ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
	}

	tk::spline s;
	s.set_points(ptsx, ptsy);
	
	for (int i = 0; i < path_size; i ++)
	{
		next_x_vals.push_back(previous_path_x[i]);
		next_y_vals.push_back(previous_path_y[i]);
	}

	// calc how to break up spline points so that we travel at our designed ref veelovity
	double target_x = 50.0;
	double target_y = s(target_x);
	// vector<double> target_sd = getFrenet(target_x, target_y, ref_yaw, map_waypoints_x, map_waypoints_y);
	// double target_s = target_sd[0];
	// vector<double> ref_sd = getFrenet(ref_x, ref_y, ref_yaw, map_waypoints_x, map_waypoints_y);
	// double ref_s = ref_sd[0];


	double target_dist = sqrt(target_x * target_x + target_y * target_y);
	double x_add_on = 0;
	for (int i = 1; i < max_path_points - path_size; i ++)
	{
		double N = target_dist / (0.02 * ref_v / 2.24);
		double x_point = i * (target_x / N);
		double y_point = s(x_point);

		double x_ref = x_point;
		double y_ref = y_point;
		double x_mapcoord = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
		double y_mapcoord = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);
		x_mapcoord += ref_x; // now x_point and y_point as the map coordinate
		y_mapcoord += ref_y;
		next_x_vals.push_back(x_mapcoord);
		next_y_vals.push_back(y_mapcoord);
	}
}


// add sensor fusion, check the other cars, lower down the ref speed 
// also decide the target lane
double path_plan_strategy5(
	vector<double> &next_x_vals, vector<double> &next_y_vals, int &lane, int last_lane,
	double car_yaw, double car_s, double car_d, double car_x, double car_y, double ref_v, double car_v,
	const vector<double> &previous_path_x, const vector<double> &previous_path_y,
	double end_path_s, double end_path_d,
	const vector<double> &map_waypoints_x,
	const vector<double> &map_waypoints_y,
	const vector<double> &map_waypoints_s,
	const vector<double> &map_waypoints_dx,
	const vector<double> &map_waypoints_dy,
	const vector<vector<double> > &sensor_fusion)
{
	// sensor_fusion
	// [id, x, y, vx, vy, s, d]
	int prev_size = previous_path_x.size();
	// if (prev_size > 0)
		// car_s = end_path_s;
	
	bool tooclose = false;
	// safe_dist will be based on the car speed
	// we given a 3 seconds response time
	double safe_dist = car_v * 1.0;
	// if (safe_dist < 30)
	// 	safe_dist = 30.0;

	cout << "lane = " << lane << ", car_v = " << car_v << ", safe dist = " << safe_dist << endl;
 	for (int i = 0; i < sensor_fusion.size(); i ++)
	{
		double obs_car_id = sensor_fusion[i][0];
		double obs_car_x  = sensor_fusion[i][1];
		double obs_car_y  = sensor_fusion[i][2];
		double obs_car_vx = sensor_fusion[i][3];
		double obs_car_vy = sensor_fusion[i][4];
		double obs_car_s  = sensor_fusion[i][5];
		double obs_car_d  = sensor_fusion[i][6];
		double obs_car_v  = sqrt(obs_car_vx*obs_car_vx + obs_car_vy*obs_car_vy);

		if (obs_car_d > lane * 4 && obs_car_d < lane * 4 + 4 )
		{
			if (obs_car_v <= car_v && obs_car_s >= car_s && obs_car_s < car_s + safe_dist)
			{
				cout << " a car in the safe_dist range, car_s = " << car_s 
					<< ", obs car s = " << obs_car_s <<  ", with speed = " << obs_car_v << endl;
				tooclose = true;
			}
			else if (obs_car_s > car_s)
			{
				double check_car_s = obs_car_s + prev_size * 0.02 * obs_car_v;
				// we will check in a futuer s range that if ego car and checked are will collision
				if (check_car_s > car_s && check_car_s - car_s < 30)
				{
					cout << " a car in a future range will collision, car_s = " 
						<< car_s << ", obs car s = " << obs_car_s << endl;
					tooclose = true;
				}
			}
		}
	}

	if (tooclose)
	{
		cout << " too close, slow down ref_v from " << ref_v << " to " << ref_v - 0.25 << endl;
		ref_v -= 0.225;
		bool lane_switch = false;
		if (lane == 0 && ref_v < 47 &&
			is_target_lane_safe(1, sensor_fusion, car_yaw, car_s, car_d, car_v, prev_size))
		{
			lane = 1;
			lane_switch = true;
			cout << "switch from lane 0 to lane 1" << endl;
		}
		else if ( lane == 2 &&  ref_v < 47 &&
			is_target_lane_safe(1, sensor_fusion, car_yaw, car_s, car_d, car_v, prev_size))
		{
			lane_switch = true;
			lane = 1;
			cout << "switch from lane 2 to lane 1" << endl;
		}
		else if (lane == 1)
		{
			if (  ref_v < 47 &&
				is_target_lane_safe(0, sensor_fusion, car_yaw, car_s, car_d, car_v, prev_size))
			{
				lane_switch = true;
				lane = 0;
				cout << "switch from lane 1 to lane 0" << endl;
			}
			else if ( ref_v < 47 &&
			 is_target_lane_safe(2, sensor_fusion, car_yaw, car_s, car_d, car_v, prev_size))
			{
				lane_switch = true;
				lane = 2;
				cout << "switch from lane 1 to lane 2" << endl;
			}
		}

	}
	else if (ref_v < 49.0 - 0.45)
	{
		ref_v += 0.225;
		cout << " speed up ref_v to " << ref_v << endl;
	}

	return ref_v;
}