#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

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

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
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

  //use the lane numbers from the Q&A video for simplicity
  int lane = 1; //lane 1 is the middle lane
  //use the reference velocity, too
  double ref_vel = 0; //mph

  string state = "KL"; //initialize in Keep Lane state


  h.onMessage([&lane, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &ref_vel, &state]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
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

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          //size of the previous path
          int prev_size = previous_path_x.size();
          //if lane change was started in the previous message wait until most of the planned trajectory is traveled, then change the lane variable
          if((state == "PLCL" || state == "PLCR") && (prev_size < 10)){
        	  if(state == "PLCL"){
        		  lane -= 1;
        	  }else {
        		  lane += 1;
        	  }
        	  state = "KL";

          }

          /*
		   * TODO: define a path made up of (x,y) points that the car will visit
		   *   sequentially every .02 seconds
		   */
          //any evaluation is possible only after a lane change is completed
          if(state != "PLCL" && state != "PLCR"){
			  if(prev_size > 0){
				  car_s = end_path_s;
			  }
			  bool too_close = false; //flag to show that a car is too close ahead
			  bool change_left = false; //flag to shwm change left is safe
			  bool change_right = false; //flag to show change right is safe
			  vector<double> costs {10, 10, 10}; //costs for KL, LCL, LCR
			  double follow_speed = 49.5; //speed to follow vehicle in lane if needed


			  for(int i = 0; i < sensor_fusion.size(); i++){
				float d = sensor_fusion[i][6]; //checking the i-th car's d position
				//for cars in our lane
				if(d < (2+4*lane+2) && d > (2+4*lane-2)){
					double vx = sensor_fusion[i][3];
					double vy = sensor_fusion[i][4];
					double check_speed = sqrt(vx*vx+vy*vy)*2.24; //car speed along the road (magnitude of the speed vector) converted to mph
					double check_car_s = sensor_fusion[i][5];

					check_car_s += ((double)prev_size*.02*check_speed/2.24); //trying to predict where the car will be based on previous data
					//and see if it is in front of us and is too close = closer than 30m
					if((check_car_s > car_s) && ((check_car_s-car_s) < 30)){
						//this car is too close
						too_close = true;
						double keep_cost = inefficiency_cost(49.5, check_speed, car_speed);
						costs[0] = keep_cost - .02; //offset to slightly favor keeping lane
						follow_speed = check_speed;
					}
				}
			  }

				vector<string> possible_states;

				//if too close decrease speed
				if(too_close){
					//check possible successor states
					possible_states = successor_states(state, lane);
					//check each possible successor state
					for(int i=0; i < possible_states.size(); i++){
						string s = possible_states[i];
						if(s.compare("KL")==0){
							state = "KL";

						}else if(s.compare("PLCL") == 0){
							int left_lane = lane - 1;
							double left_lane_speed;
							bool left_clear = is_lane_clear(sensor_fusion, left_lane, car_s, prev_size, left_lane_speed);
							if(left_clear){

								costs[1] = inefficiency_cost(49.5, left_lane_speed, car_speed);
								change_left = true;

							}
						}else if(s.compare("PLCR") == 0){
							int right_lane = lane + 1;
							double right_lane_speed;
							bool right_clear = is_lane_clear(sensor_fusion, right_lane, car_s, prev_size, right_lane_speed);
							if(right_clear){

								costs[2] = inefficiency_cost(49.5, right_lane_speed, car_speed);
								change_right = true;
							}
						}
					}
				}
				else if(ref_vel < 49){
					ref_vel += .224; //normal acceleration
				}
				//get the minimum index item of the costs vector
				vector<double>::iterator result = min_element(costs.begin(), costs.end());
				int minCostIndex = std::distance(costs.begin(), result);
				int new_lane = lane;

				//perform the lane change
				if(change_left && (minCostIndex == 1)){
					if(ref_vel < 45){
						ref_vel+= .2; //acceleration before lane change if speed is low
					}
					new_lane -= 1;
					state = "PLCL";
				}else if(change_right && (minCostIndex == 2)){
					if(ref_vel < 45){
						ref_vel+= .2; //acceleration before lane change if speed is low
					}
					new_lane += 1;
					state = "PLCR";
				}else if(ref_vel > follow_speed){
						ref_vel -= 0.224; //deceleration to follow vehicle in lane
				}


			  //define the actual points to be set for the spline
			  vector<double> next_x_vals;
			  vector<double> next_y_vals;

			  generate_trajectory(car_x,car_y,car_yaw,car_s,ref_vel,new_lane,prev_size,previous_path_x,previous_path_y,map_waypoints_s,map_waypoints_x,map_waypoints_y,next_x_vals,next_y_vals);


			  msgJson["next_x"] = next_x_vals;
			  msgJson["next_y"] = next_y_vals;


			  auto msg = "42[\"control\","+ msgJson.dump()+"]";

			  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          } //end of lane change checking if
          else{
        	  msgJson["next_x"] = previous_path_x; //while lane change is not complete the same trajectory is resent
			  msgJson["next_y"] = previous_path_y; //(decreased with the already traveled parts)


			  auto msg = "42[\"control\","+ msgJson.dump()+"]";

			  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }

        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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
