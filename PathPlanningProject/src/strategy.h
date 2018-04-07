#include <vector>
using namespace std;

extern void path_plan_strategy1(vector<double> &next_x_vals, vector<double> &next_y_vals, 
	double car_yaw, double car_x, double car_y, double car_v);

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
	vector<double> &next_x_vals, vector<double> &next_y_vals, int current_lane, int target_lane,
	double car_yaw, double car_s, double car_d, double car_x, double car_y, double ref_v,
	vector<double> &previous_path_x, vector<double> &previous_path_y,
	vector<double> &map_waypoints_x,
	vector<double> &map_waypoints_y,
	vector<double> &map_waypoints_s,
	vector<double> &map_waypoints_dx,
	vector<double> &map_waypoints_dy);

extern double path_plan_strategy5(
	vector<double> &next_x_vals, vector<double> &next_y_vals, int &target_lane, int last_lane,
	double car_yaw, double car_s, double car_d, double car_x, double car_y, double ref_v, double car_v,
	const vector<double> &previous_path_x, const vector<double> &previous_path_y ,
	double end_path_s, double end_path_d,
	const vector<double> &map_waypoints_x,
	const vector<double> &map_waypoints_y,
	const vector<double> &map_waypoints_s,
	const vector<double> &map_waypoints_dx,
	const vector<double> &map_waypoints_dy,
	const vector<vector<double> > &sensor_fusion);

