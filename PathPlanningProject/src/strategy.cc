#include <vector>
#include <math.h>
#include <assert.h>

using namespace std;
extern double deg2rad(double x);
constexpr double pi() { return M_PI; }

void path_plan_strategy1(vector<double> &next_x_vals, vector<double> &next_y_vals, 
	double car_yaw, double car_x, double car_y)
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

	for (int i = 0; i < 50 - path_size; i ++)
	{
		next_x_vals.push_back(pos_x + dist_inc * cos(angle + (i + 1) * pi() / 100));
		next_y_vals.push_back(pos_y + dist_inc * sin(angle + (i + 1) * pi() / 100));
		pos_x += dist_inc * cos(angle + (i + 1) * pi() / 100);
		pos_y += dist_inc * sin(angle + (i + 1) * pi() / 100);
	}
}

