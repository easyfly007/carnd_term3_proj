#include <vector>
#include <math.h>

using namespace std;
extern double deg2rad(double x);

void path_plan_strategy1(vector<double> &next_x_vals, vector<double> &next_y_vals, double car_yaw, double car_x, double car_y)
{
	// input: car_yaw
	// output: next_x_vals, next_y_vals

	// make the car move in a straight line
	double dist_inc = 0.5;
	for (int i = 0; i < 50; i ++)
	{
		next_x_vals.push_back(car_x + (dist_inc * i) * cos(deg2rad(car_yaw)) );
		next_y_vals.push_back(car_y + (dist_inc * i) * sin(deg2rad(car_yaw)));
	}
	return;
}
