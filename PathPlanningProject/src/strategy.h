#include <vector>
using namespace std;

extern void path_plan_strategy1(vector<double> &next_x_vals, vector<double> &next_y_vals, 
	double car_yaw, double car_x, double car_y);

extern void path_plan_strategy2(vector<double> &next_x_vals, vector<double> &next_y_vals, 
	double car_yaw, double car_x, double car_y, 
	vector<double> &previous_path_x, vector<double> &previous_path_y );

extern void path_plan_strategy3(vector<double> &next_x_vals, vector<double> &next_y_vals, 
	double car_yaw, double car_x, double car_y, 
	vector<double> &previous_path_x, vector<double> &previous_path_y,
	vector<double> &map_waypoints_x,
	vector<double> &map_waypoints_y,
	vector<double> &map_waypoints_s,
	vector<double> &map_waypoints_dx,
	vector<double> &map_waypoints_dy);

extern void path_plan_strategy4(
	vector<double> &next_x_vals, vector<double> &next_y_vals, 
	double car_yaw, double car_d, double car_x, double car_y, 
	vector<double> &previous_path_x, vector<double> &previous_path_y,
	vector<double> &map_waypoints_x,
	vector<double> &map_waypoints_y,
	vector<double> &map_waypoints_s,
	vector<double> &map_waypoints_dx,
	vector<double> &map_waypoints_dy);

