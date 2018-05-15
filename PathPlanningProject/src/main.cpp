#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "../../Eigen-3.3/Eigen/Core"
#include "../../Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

using namespace std;

// for convenience
using json = nlohmann::json;


// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
	auto found_null = s.find("null");
	auto b1 = s.find_first_of("[");
	auto b2 = s.find_first_of("}");
	if (found_null != string::npos) {
		return "";
	} else if (b1 != string::npos && b2 != string::npos) {
		return s.substr(b1, b2 - b1 + 2);
	}
	return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
	angle = min(2*pi() - angle, angle);

	if(angle > pi()/4)
	{
		closestWaypoint++;
		if (closestWaypoint == maps_x.size())
		{
			closestWaypoint = 0;
		}
	}

	return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, 
	const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, 
	const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

const int max_path_points = 80;

int getLane(double d)
{
	if (0.0 <= d && d <= 4.0)
		return 0;
	if (4.0 <= d && d <= 8.0)
		return 1;
	return 2;
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

		if (obs_car_d > target_lane * 4 && obs_car_d < target_lane * 4 + 4 )
		{
			if (obs_car_s < car_s + car_v && obs_car_s > car_s - 10)
			{
				cout << "try to switch lane, target_lane= " << target_lane << " not safe, a car in range" << endl;
				return false; 
			}
			
			if (obs_car_v <= car_v + 0.2)
			{
				if ( obs_car_s >= car_s - 5.0 && obs_car_s < car_s + safe_dist_ahead)
				{
					cout << "try to switch lane, target_lane= " << target_lane << " not safe" << endl;
					return false;
				}
			}


			if (obs_car_v >= car_v - 0.2)
			{
				if (obs_car_s <= car_s + 5.0 && obs_car_s > car_s - 4.0 * (obs_car_v - car_v + 0.2))
				{
					cout << "try to switch lane, target lane = " << target_lane << " not safe" << endl;
					return false;
				}
			}
		}		
	}
	return true;
}


// smooth the line
void buildTrajectory(
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
	double previous_path_length = path_size * ref_v *0.02;
	vector<double> next_wp0 = getXY(car_s + previous_path_length + ref_v * 0.5, 4 * (target_lane* 0.9 + 0.1 * current_lane) + 2,
		map_waypoints_s, map_waypoints_x,map_waypoints_y);
	vector<double> next_wp1 = getXY(car_s + previous_path_length + ref_v * 2, 4 * target_lane + 2, 
		map_waypoints_s, map_waypoints_x,map_waypoints_y);
	vector<double> next_wp2 = getXY(car_s + previous_path_length + ref_v * 3, 4 * target_lane + 2, 
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
	cout << "set_spline ptsx as " << endl;
	for (int j = 0; j < ptsx.size(); j ++)
	{	
		cout << ptsx[j] << " ";
	}
	cout << endl;
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
double behaviorControl(
	vector<double> &next_x_vals, vector<double> &next_y_vals, int &lane, int last_lane,
	double car_yaw, double car_s, double car_d, double car_x, double car_y, double ref_v,
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
	
	bool tooclose = false;
	// safe_dist will be based on the car speed
	// we given a 3 seconds response time
	double safe_dist = ref_v * 1.0 + 5.0;
	cout << "lane = " << lane << ", ref_v = " << ref_v << ", safe dist = " << safe_dist << endl;
	double front_car_v = 0.;
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
			if (obs_car_v <= ref_v && obs_car_s >= car_s && obs_car_s < car_s + safe_dist)
			{
				cout << " a car in the safe_dist range, car_s = " << car_s 
					<< ", obs car s = " << obs_car_s <<  ", with speed = " << obs_car_v << endl;
				tooclose = true;
				front_car_v = obs_car_v;
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
					front_car_v = obs_car_v;
				}
			}
		}
	}

	if (tooclose)
	{
		bool lane_switch = false;
		if (lane == 0 && ref_v < 47 &&
			is_target_lane_safe(1, sensor_fusion, car_yaw, car_s, car_d, ref_v, prev_size))
		{
			lane = 1;
			lane_switch = true;
			cout << "switch from lane 0 to lane 1" << endl;
		}
		else if ( lane == 2 &&  ref_v < 47 &&
			is_target_lane_safe(1, sensor_fusion, car_yaw, car_s, car_d, ref_v, prev_size))
		{
			lane_switch = true;
			lane = 1;
			cout << "switch from lane 2 to lane 1" << endl;
		}
		else if (lane == 1)
		{
			if (  ref_v < 47 &&
				is_target_lane_safe(0, sensor_fusion, car_yaw, car_s, car_d, ref_v, prev_size))
			{
				lane_switch = true;
				lane = 0;
				cout << "switch from lane 1 to lane 0" << endl;
			}
			else if ( ref_v < 47 &&
			 is_target_lane_safe(2, sensor_fusion, car_yaw, car_s, car_d, ref_v, prev_size))
			{
				lane_switch = true;
				lane = 2;
				cout << "switch from lane 1 to lane 2" << endl;
			}
		}
		if (lane_switch == false)
		{
			// in case there's a slow care in fron of us and the target lane is not safe,
			// slow down more to avoid collision
			double ref_old = ref_v;
			double speed_diff = ref_v - front_car_v;
			ref_v -= 0.225;
			ref_v -= speed_diff * 0.05;
			cout << "ref_v switch from " << ref_old << " to " << ref_v << endl;
		}
		
	}
	else if (ref_v < 49.0)
	{
		ref_v += 0.25;
		cout << " speed up ref_v to " << ref_v << endl;
	}

	return ref_v;
}




double ref_v = 1.0;
int lane = 1;
int last_lane = 1;

int main() {
	uWS::Hub h;


	// Load up map values for waypoint's x,y,s and d normalized normal vectors
	vector<double> map_waypoints_x;
	vector<double> map_waypoints_y;
	vector<double> map_waypoints_s;
	vector<double> map_waypoints_dx;
	vector<double> map_waypoints_dy;

	// Waypoint map to read from
	string map_file_ = "../data/highway_map.csv";
	// The max s value before wrapping around the track back to 0
	double max_s = 6945.554;

	ifstream in_map_(map_file_.c_str(), ifstream::in);

	string line;
	while (getline(in_map_, line)) {
		istringstream iss(line);
		double x;
		double y;
		float s;
		float d_x;
		float d_y;
		iss >> x;
		iss >> y;
		iss >> s;
		iss >> d_x;
		iss >> d_y;
		map_waypoints_x.push_back(x);
		map_waypoints_y.push_back(y);
		map_waypoints_s.push_back(s);
		map_waypoints_dx.push_back(d_x);
		map_waypoints_dy.push_back(d_y);
	}

	h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
				uWS::OpCode opCode) {
			// "42" at the start of the message means there's a websocket message event.
			// The 4 signifies a websocket message
			// The 2 signifies a websocket event
			//auto sdata = string(data).substr(0, length);
			//cout << sdata << endl;
		cout << endl << endl;
			if (length && length > 2 && data[0] == '4' && data[1] == '2') {

			auto s = hasData(data);

			if (s != "") {
			auto j = json::parse(s);

			string event = j[0].get<string>();

			if (event == "telemetry") {
			// j[1] is the data JSON object

			// Main car's localization Data
			double car_x = j[1]["x"];
			double car_y = j[1]["y"];
			double car_s = j[1]["s"];
			double car_d = j[1]["d"];
			double car_yaw = j[1]["yaw"];
			double car_speed = j[1]["speed"];

			// Previous path data given to the Planner
			vector<double> previous_path_x = j[1]["previous_path_x"];
			vector<double> previous_path_y = j[1]["previous_path_y"];
			// Previous path's end s and d values 
			double end_path_s = j[1]["end_path_s"];
			double end_path_d = j[1]["end_path_d"];

			// Sensor Fusion Data, a list of all other cars on the same side of the road.
			vector<vector<double> > sensor_fusion = j[1]["sensor_fusion"];

			json msgJson;

			vector<double> next_x_vals;
			vector<double> next_y_vals;


			// path_plan_strategy1(next_x_vals, next_y_vals, car_yaw, car_x, car_y, car_speed);
			// path_plan_strategy2(next_x_vals, next_y_vals, car_yaw, car_x, car_y, 
			// 	previous_path_x, previous_path_y);
			// path_plan_strategy3(next_x_vals, next_y_vals, car_yaw, car_x, car_y, 
			// 	previous_path_x, previous_path_y,
			// 	map_waypoints_x, map_waypoints_y,map_waypoints_s, map_waypoints_dx, map_waypoints_dy);
			if (ref_v < 0.0)
				ref_v = car_speed;

			static int lane_switch_cnt = 0;
			double last_v = ref_v;

			last_lane = lane;
			ref_v =  behaviorControl(
				next_x_vals, next_y_vals, lane, last_lane,
				car_yaw, car_s, car_d, car_x, car_y, ref_v,
				previous_path_x, previous_path_y, end_path_s, end_path_d, map_waypoints_x,
				map_waypoints_y, map_waypoints_s, map_waypoints_dx, map_waypoints_dy, sensor_fusion);
			
			if (last_lane != lane)
				lane_switch_cnt ++;
			else
				lane_switch_cnt = 0;
			if (lane_switch_cnt > 1)
			{
				// avoid continous lane changing
				cout << "keep lane, can not continously changing lane" << endl;
				ref_v = last_v;
				lane = last_lane;
				lane_switch_cnt = 0;
			}

			buildTrajectory(
				next_x_vals, next_y_vals, last_lane, lane, 
				car_yaw, car_s, car_d, car_x, car_y, ref_v,
				previous_path_x, previous_path_y,
				map_waypoints_x, map_waypoints_y, map_waypoints_s,map_waypoints_dx, map_waypoints_dy);


			msgJson["next_x"] = next_x_vals;
			msgJson["next_y"] = next_y_vals;

			auto msg = "42[\"control\","+ msgJson.dump()+"]";

			//this_thread::sleep_for(chrono::milliseconds(1000));
			ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

			}
			} else {
				// Manual driving
				std::string msg = "42[\"manual\",{}]";
				ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
			}
			}
	});

	// We don't need this since we're not using HTTP but if it's removed the
	// program
	// doesn't compile :-(
	h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
				size_t, size_t) {
			const std::string s = "<h1>Hello world!</h1>";
			if (req.getUrl().valueLength == 1) {
			res->end(s.data(), s.length());
			} else {
			// i guess this should be done more gracefully?
			res->end(nullptr, 0);
			}
			});

	h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
			std::cout << "Connected!!!" << std::endl;
			});

	h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
				char *message, size_t length) {
			ws.close();
			std::cout << "Disconnected" << std::endl;
			});

	int port = 4567;
	if (h.listen(port)) {
		std::cout << "Listening to port " << port << std::endl;
	} else {
		std::cerr << "Failed to listen to port" << std::endl;
		return -1;
	}
	h.run();
}
