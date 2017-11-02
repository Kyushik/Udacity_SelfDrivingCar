#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"

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

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
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
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
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
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

            vector<double> next_x_bezier;
            vector<double> next_y_bezier;


          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
            // Number of path planning points
            double dist_inc = 0.45;
            double num_points = 50;
            int road_index;

            double num_bezier_points = 5;

            ////////////////////////////////////////////////// Mode Selection //////////////////////////////////////////////////
            std::string mode;
            mode = "Lane_Keeping";
            // 
            // if (car_s <= 300) {
            //    mode = "Lane_Keeping";
            // } else if (car_s <= 400) {
            //    mode = "Lane_Change_Left";
            // } else if (car_s <= 450){
            //   mode = "Lane_Keeping";
            // } else if (car_s <= 500){
            //   mode = "Lane_Change_Right";
            // } else if (car_s <= 600){
            //   mode = "Lane_Keeping";
            // } else if (car_s <= 700){
            //   mode = "Lane_Change_Right";
            // } else if (car_s <= 800){
            //   mode = "Lane_Keeping";
            // } else if (car_s <= 900){
            //   mode = "Lane_Change_Left";
            // } else if (car_s <= 1000){
            //   mode = "Lane_Keeping";
            // } else if (car_s <= 1100){
            //   mode = "Lane_Change_Left";
            // } else {
            //   mode = "Lane_Keeping";
            // }

            ////////////////////////////////////////////////// Path Generation //////////////////////////////////////////////////
            // Get current lane
            int current_lane = 0;
            if (car_d <= 4) {
              current_lane = 1; // First lane
            } else if (car_d <= 8) {
              current_lane = 2;
            } else {
              current_lane = 3;
            }

            double target_s = car_s + num_points * dist_inc;
            double target_d;

            double check_start_lane_change;
            double target_end_d;

            // Lane Keeping
            if (mode == "Lane_Keeping"){
              if (current_lane == 1) {
                target_d = 2;
              } else if(current_lane == 2) {
                target_d = 6;
              } else {
                target_d = 10;
              }
              check_start_lane_change = 0;

            // Lane Change to Left
            } else if (mode == "Lane_Change_Left") {
              // Select Target Lane
              if (check_start_lane_change == 0) {
                if (current_lane == 2) {
                  target_end_d = 2;
                } else {
                  target_end_d = 6;
                }
                check_start_lane_change = 1;
              }

              // Lane Change
              if (end_path_d > target_end_d){
                target_d = target_d - 0.1;
              }
            // Lane Change to Right
          } else {
            // Select Target Lane
            if (check_start_lane_change == 0) {
              if (current_lane == 1) {
                target_end_d = 6;
              } else {
                target_end_d = 10;
              }
              check_start_lane_change = 1;
            }

            // Lane Change
            if (end_path_d < target_end_d){
              target_d = target_d + 0.1;
            }
          }

            vector<double> target_xy;
            target_xy = getXY(target_s, target_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);

            double pos_x;
            double pos_y;
            double angle;
            int path_size = previous_path_x.size();

            for(int i = 0; i < path_size; i++)
            {
                next_x_vals.push_back(previous_path_x[i]);
                next_y_vals.push_back(previous_path_y[i]);
            }

            if(path_size == 0)
            {
                pos_x = car_x;
                pos_y = car_y;
                angle = deg2rad(car_yaw);
            }
            else
            {
                pos_x = previous_path_x[path_size-1];
                pos_y = previous_path_y[path_size-1];

                double pos_x2 = previous_path_x[path_size-2];
                double pos_y2 = previous_path_y[path_size-2];
                angle = atan2(pos_y-pos_y2,pos_x-pos_x2);
            }

            int num_update = num_points - path_size;
            double dx = (target_xy[0] - pos_x)/(num_update);
            double dy = (target_xy[1] - pos_y)/(num_update);
            for(int i = 0; i < num_points - path_size; i++)
            {
                next_x_vals.push_back(pos_x + dx);
                next_y_vals.push_back(pos_y + dy);
                pos_x += dx;
                pos_y += dy;
            }

            ////////////////////////////////////////////////// Print Information //////////////////////////////////////////////////
            if (car_speed > 40) {
              std::cout << "car_s: " << car_s << std::endl;
              std::cout << "car_d: " << car_d << std::endl;
              std::cout << "target_d: " << target_d << std::endl;
              std::cout << "mode: " << mode << std::endl;
              std::cout << "Car Speed: " << car_speed << std::endl;
              std::cout << std::endl;
            }

            ////////////////////////////////////////////////// Smoothing by Bezier Curve //////////////////////////////////////////////////
            // Smoothing the Vehicle Using Bezier Curve
            double t = 0;
            double delta_t_bezier = 1 / (num_points - 1);
            double bezier_x;
            double bezier_y;

            for(int i = 0; i < num_points; i++) {

              bezier_x = pow((1-t), 5) * next_x_vals[0] + 5 * pow((1-t), 4) * t * next_x_vals[9] + 10 * pow((1-t), 3) * pow(t,2) * next_x_vals[19] + 10 * pow((1-t), 2) * pow(t,3) * next_x_vals[29] + 5 * (1-t) * pow(t,4) * next_x_vals[39] + pow(t,5) * next_x_vals[49];
              bezier_y = pow((1-t), 5) * next_y_vals[0] + 5 * pow((1-t), 4) * t * next_y_vals[9] + 10 * pow((1-t), 3) * pow(t,2) * next_y_vals[19] + 10 * pow((1-t), 2) * pow(t,3) * next_y_vals[29] + 5 * (1-t) * pow(t,4) * next_y_vals[39] + pow(t,5) * next_y_vals[49];

              next_x_bezier.push_back(bezier_x);
              next_y_bezier.push_back(bezier_y);

              t = t + delta_t_bezier;
            }

            // for(int i = 1; i < num_points; i++){
            //   std::cout << "Next_x_vals: " << next_x_vals[i] << " / " << "Next_x_bezier: " << next_x_bezier[i] << std::endl;
            //   std::cout << "Next_y_vals: " << next_y_vals[i] << " / " << "Next_y_bezier: " << next_y_bezier[i] << std::endl;
            //   // double next_x_diff_squre = (next_x_vals[i] - next_x_vals[i-1]) * (next_x_vals[i] - next_x_vals[i-1]);
            //   // double next_y_diff_squre = (next_y_vals[i] - next_y_vals[i-1]) * (next_y_vals[i] - next_y_vals[i-1]);
            //   // std::cout << "diff_vals: " << sqrt(next_x_diff_squre + next_y_diff_squre) << std::endl;
            // }

            // std::cout << std::endl;

            next_x_vals = next_x_bezier;
            next_y_vals = next_y_bezier;

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
