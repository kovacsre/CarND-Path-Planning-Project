#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <vector>
#include "spline.h"

// for convenience
using std::string;
using std::vector;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
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

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, 
                    const vector<double> &maps_y) {
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); ++i) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, 
                 const vector<double> &maps_y) {
  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y-y),(map_x-x));

  double angle = fabs(theta-heading);
  angle = std::min(2*pi() - angle, angle);

  if (angle > pi()/2) {
    ++closestWaypoint;
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, 
                         const vector<double> &maps_x, 
                         const vector<double> &maps_y) {
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if (next_wp == 0) {
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

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; ++i) {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, 
                     const vector<double> &maps_x, 
                     const vector<double> &maps_y) {
  int prev_wp = -1;

  while (s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1))) {
    ++prev_wp;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),
                         (maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};
}

/*Calculate cost of a lane change from speed point of view
 * inputs:  target speed - the speed we want to reach
 * 			check_speed - the speed of the lane we want to change to
 * 			current_speed - our current speed
 */
double inefficiency_cost(double target_speed, double check_speed, double current_speed) {
	double cost = 1 - (check_speed / target_speed);
	return cost;
}

/*Generate trajectory using spline.h
 *
 */
void generate_trajectory(double car_x, double car_y, double car_yaw, double car_s, double ref_vel,
						 int lane, int prev_size, vector<double> &previous_path_x, vector<double> &previous_path_y,
						 vector<double> &map_waypoints_s, vector<double> &map_waypoints_x, vector<double> &map_waypoints_y,
						 vector<double> &next_x_vals, vector<double> &next_y_vals) {


	//define widely spaced x,y points along a bigger distance -now 30m
	vector<double> ptsx;
	vector<double> ptsy;
	//reference x,y,yaw sates to see where the car is
	//either use the reference points or the last point of the previous path
	double ref_x = car_x;
	double ref_y = car_y;
	double ref_yaw = deg2rad(car_yaw);

	//if previous path size is almost empty use the reference state
	if(prev_size < 2){
	  //use two points that make the path tangent to the car
	  double prev_car_x = car_x - cos(car_yaw);
	  double prev_car_y = car_y - sin(car_yaw);

	  ptsx.push_back(prev_car_x);
	  ptsx.push_back(car_x);

	  ptsy.push_back(prev_car_y);
	  ptsy.push_back(car_y);
	}
	else{
	  //reference state is the end point of the previous path
	  ref_x = previous_path_x[prev_size-1];
	  ref_y = previous_path_y[prev_size-1];

	  //use two points that make the path tangent to the previous path end points
	  double ref_x_prev = previous_path_x[prev_size-2];
	  double ref_y_prev = previous_path_y[prev_size-2];
	  ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

	  ptsx.push_back(ref_x_prev);
	  ptsx.push_back(ref_x);

	  ptsy.push_back(ref_y_prev);
	  ptsy.push_back(ref_y);
	}
	//add 3 points ahead of the vehicle 30 ms one after the other in Frenet coordinates
	vector<double> next_wp0 = getXY(car_s+30,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
	vector<double> next_wp1 = getXY(car_s+60,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
	vector<double> next_wp2 = getXY(car_s+90,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);

	ptsx.push_back(next_wp0[0]);
	ptsx.push_back(next_wp1[0]);
	ptsx.push_back(next_wp2[0]);

	ptsy.push_back(next_wp0[1]);
	ptsy.push_back(next_wp1[1]);
	ptsy.push_back(next_wp2[1]);

	//transform to the car's local coordinates = last point of previous path is (x,y) = (0,0) yaw is 0, too
	for (int i=0; i < ptsx.size(); i++){
	  //shift car reference angle to 0 degrees
	  double shift_x = ptsx[i]-ref_x;
	  double shift_y = ptsy[i]-ref_y;

	  ptsx[i] = (shift_x*cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
	  ptsy[i] = (shift_x*sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
	}

	//create a spline
	tk::spline s;

	//set x,y points to the spline
	s.set_points(ptsx,ptsy);

	//start with the previous path points from last time
	for(int i=0; i<previous_path_x.size(); i++){
	  next_x_vals.push_back(previous_path_x[i]);
	  next_y_vals.push_back(previous_path_y[i]);

	}

	//calculate how to break up the spline so that the car travels with the intended reference velocity
	double target_x = 30.0;
	double target_y = s(target_x);
	double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));

	double x_add_on = 0;

	//fill up the rest of the points of the path planner after adding the remaining points of the previous path
	for(int i =1; i<=50-previous_path_x.size(); i++){

	  double N = (target_dist/(.02*ref_vel/2.24)); //number of path points to be generated on the spline
	  double x_point = x_add_on+(target_x)/N;
	  double y_point = s(x_point);

	  x_add_on = x_point;

	  double x_ref = x_point;
	  double y_ref = y_point;

	  //returning to "normal" coordinates: rotation back
	  x_point = (x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw));
	  y_point = (x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw));
	  //shift back
	  x_point += ref_x;
	  y_point += ref_y;

	  next_x_vals.push_back(x_point);
	  next_y_vals.push_back(y_point);
	}

}

/*Choose next state depending on current state and state machine
 *
 */
vector<string> successor_states(string state, int lane) {
	vector<string> next_states;
	next_states.push_back("KL");
	if(state.compare("KL")==0){
		if(lane > 0){
			next_states.push_back("PLCL");
		}
		if(lane < 2){
			next_states.push_back("PLCR");
		}
	}else if(state.compare("PLCL")==0){
		if(lane > 0){
			//next_states.push_back("PLCL");
			next_states.push_back("LCL");
		}
	}else if(state.compare("PLCR")==0){
		if (lane < 2){
			//next_states.push_back("PLCR");
			next_states.push_back("LCR");
		}
	}
	return next_states;
}

/*Check if the lane we want to change to is clear
 * and calculate its speed
 */
bool is_lane_clear(vector<vector<double>> &sensor_fusion, int lane_to_check, double car_s, int prev_size, double &lane_speed){
	bool way_is_clear = true;
	double sum_speed=0;
	int num_of_cars = 0;

	for(int i = 0; i < sensor_fusion.size(); i++){
	  float d = sensor_fusion[i][6]; //checking the i-th car's d position

	  //for cars in the lane lane to check
	  if(d < (2+4*lane_to_check+2) && d > (2+4*lane_to_check-2)){
		  double vx = sensor_fusion[i][3];
		  double vy = sensor_fusion[i][4];
		  double check_speed = sqrt(vx*vx+vy*vy)*2.24; //car speed along the road (magnitude of the speed vector) + convert to mph
		  double check_car_s = sensor_fusion[i][5];

		  check_car_s += ((double)prev_size*.02*check_speed/2.24); //trying to predict where the car will be based on velocity

		  //if it will be close (from a little bit behind to a little bit ahead) then the lane is not clear
		  if((check_car_s >= car_s - 7) && ((check_car_s-car_s) < 15)){
			  way_is_clear = false;
		  }

		  //calculate the average speed of the vehicles in sight in the lane
		  if((check_car_s > car_s) && ((check_car_s - car_s) < 50)){
			  sum_speed += check_speed;
			  num_of_cars += 1;
		  }
	  }
	}
	if(num_of_cars > 0){
		lane_speed = sum_speed/num_of_cars;
	}else{
		lane_speed = 49.5;
	}
	return way_is_clear;
}
#endif  // HELPERS_H
