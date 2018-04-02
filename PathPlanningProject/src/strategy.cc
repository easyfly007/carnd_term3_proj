vector<vector<double> > path_plan_strategy1()
{

double dist_inc = 0.5;

	// strategy 4, check if there's car in the front of the ego car 
	// slow down if there's a car in the front of us

	/* 
	// strategy 3, keep in the lane
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
	for (int i = 0; i < 50; i ++)
	{
		double next_s = pos_s + (i + 1) * dist_inc;
		double next_d = pos_d;
		vector<double> next_xy = getXY(next_s, next_d, map_waypoints_s,map_waypoints_x,map_waypoints_y);
		double next_x = next_xy[0];
		double next_y = next_xy[1];
		next_x_vals.push_back(next_x);
		next_y_vals.push_back(next_y);
	}
	*/


	/*
	// strategy 2, consider the previous points and calc the car angle manually
	// keep the car turing around in a circle 
	double pos_x, pos_y;
	double angle;
	int path_size = previous_path_x.size();
	assert(previous_path_x.size() == previous_path_y.size());

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
	*/

	/* strategy 1, go straight line
	for (int i = 0; i < 50; i ++)
	{
		next_x_vals.push_back(car_x + (dist_inc * i) * cos(deg2rad(car_yaw)) );
		next_y_vals.push_back(car_y + (dist_inc * i) * sin(deg2rad(car_yaw)));
	}
	*/
}
