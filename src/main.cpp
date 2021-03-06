#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <string>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include <map>
#include <iterator>
#include "helpers.h"
#include "constant.h"
#include "vehicle.h"
#include "prediction.h"
#include "target.h"
#include "trajectory.h"

using namespace std;

// for convenience
using json = nlohmann::json;

//ego vehicle
Vehicle ego;

//reference velocity to target
double ref_vel = 0.0; //mph 
double ref_a = 0.0;//mps2

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  //ego vehicle
  ego = Vehicle(0, 1, 0.0, 0.0, ref_vel, ref_a);

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  ifstream in_map_ (map_file_.c_str(), ifstream::in );

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

            int prev_size = previous_path_x.size();

            if(prev_size > 0)
            {
              car_s = end_path_s;
            }

            // cout<<"\nVehicle starting position:\n";
            // cout<<"car_x: "<<car_x<<"\tcar_y: "<<car_y<<"\tcar_s: "<<car_s<<"\tcar_d: "<<car_d<<"\tcar_yaw: "<<car_yaw<<"\tcar_speed: "<<car_speed<<"\n";

            // bool too_close = false;

            //update ego vehicle info
            ego.l = get_lane(car_d);
            ego.s = car_s;
            ego.d = car_d;
            ego.v = mph2mps(car_speed);

            // //get vehicles from sensor fusion data within FOV
            // map<int, Vehicle> vehicles = get_vehicle_in_FOV(sensor_fusion,ego);
            Prediction prediction = Prediction(sensor_fusion,ego);
            //find ref_v to use
            //generate trajectory from sensor fusion data
            // cout<<"/ndebug sensor fusion data!\n\n";

            // for(int i=0; i< sensor_fusion.size();i++)
            // {
            //   double s = sensor_fusion[i][5]; 
            //   // only use vehicles within FOV: 50m
            //   if(fabs(s) > FOV)
            //   {
            //     continue;
            //   }
            //   double d = sensor_fusion[i][6];
            //   int l = get_lane(d);
            //   if( l > 2 || l < 0)
            //   {
            //     continue;
            //   }
            //   //car is in my lane
            //   double id = sensor_fusion[i][0];
            //   // double x = sensor_fusion[i][1];
            //   // double y = sensor_fusion[i][2];
            //   double vx = sensor_fusion[i][3];
            //   double vy = sensor_fusion[i][4];
            //   double lane_speed = sqrt(vx*vx + vy*vy);

            //   Vehicle vehicle = Vehicle(l,s,lane_speed,0);
            //   vehicle.state = CS;
            //   vehicles.insert(std::pair<int,Vehicle>(id,vehicle));

            //   cout<<"id: "<<id<<"\ts: "<<s<<"\td: "<<d<<"\tlane_speed: "<<lane_speed<<"\n";

            //   // float d = sensor_fusion[i][6];
            //   // if(d < (2+4*lane+2) && d > (2+4*lane - 2))
            //   // {
            //   //   double vx = sensor_fusion[i][3];
            //   //   double vy = sensor_fusion[i][4];
            //   //   double check_speed = sqrt(vx*vx + vy*vy);
            //   //   double check_car_s = sensor_fusion[i][5];

            //   //   check_car_s += ((double)prev_size*.02*check_speed);
            //   //   //check s values greater than mine and s gap
            //   //   if((check_car_s > car_s) && ((check_car_s - car_s) < 30))
            //   //   {
            //   //     //do some logic here , lower reference velocity so we dont crash into the cat in front of un
            //   //     //could also flag to try to change lanes
            //   //     // ref_vel = 29.5; // mph
            //   //     too_close = true;
            //   //     if(lane > 0)
            //   //     {
            //   //       lane = 0;
            //   //     }
            //   //   }
            //   // }
            // }
            // cout<<"\n";

            //generate predictions
            //TODO

            map<int ,vector<Vehicle>> predictions;
            prediction.predict(predictions);

            // map<int, Vehicle>::iterator it = vehicles.begin();
            // while(it != vehicles.end())
            // {
            //     int v_id = it->first;
            //     vector<Vehicle> preds = it->second.generate_predictions();
            //     predictions[v_id] = preds;
            //     it++;
            // }
            // cout<<"\ndebug predictions!\n";
            // for(map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); it++)
            // {
            //   vector<Vehicle> m_vehicles = it->second;
            //   cout<<"id: "<<it->first<<"\tsize: "<< m_vehicles.size();
            //   for(int i=0; i< m_vehicles.size(); i++)
            //   {
            //     cout<<"\ns ==> "<<m_vehicles[i].s<<"\tlane: "<<m_vehicles[i].l<<"\nv: "<<m_vehicles[i].v;
            //   }
            //   cout<<"\n\n";
            // }
            //Generate trajectory based on predictions
            Target get_target{predictions, ego};
            Vehicle target;
            get_target.get_target_vehicle(target);
            cout<<"\ntarget para=> s: "<<target.s<<"\td: "<<target.d<<"\tv: "<<target.v<<"\ta: "<<target.a<<"\tlane: "<<target.l<<"\n";

            // ego.realize_next_state(trajectory);
            // vector<vector<double>> previous_xy;
            // for(int i=0; i< previous_path_x.size(); i++)
            // {
            //   double px = previous_path_x[i];
            //   double py = previous_path_y[i];
            //   previous_xy.push_back(vector<double>(px, py));
            // }
            // vector<double> delta{0,0,0,0,0,0};//offset which the ideal car should be between the surronding
            // Trajectory traj = Trajectory( ego,  target, delta, previous_xy);

            // vector<vector<double>> new_traj = traj.generate_trajectory();

            // ref_vel = ego.v;
            // ref_a = ego.a;

            // ifose)(too_cl
            // {
            //   ref_vel -= .224;
            // }
            // else if(ref_vel < 49.5)
            // {
            //   ref_vel += .224;
            // }

            vector<double> ptsx;
            vector<double> ptsy;

            //reference x, y, yaw
            double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = deg2rad(car_yaw);

            //if the previous size is almost empty, use the car as starting reference
            if(prev_size < 2)
            {
              //use two points that make the path tangent to the car
              double pre_car_x = car_x - cos(car_yaw);
              double pre_car_y = car_y - sin(car_yaw);

              ptsx.push_back(pre_car_x);
              ptsx.push_back(car_x);

              ptsy.push_back(pre_car_y);
              ptsy.push_back(car_y);
            }//use previous path's end points as starting reference
            else
            {

              //redefine reference state as previous path end points 
              ref_x = previous_path_x[prev_size-1];
              ref_y = previous_path_y[prev_size-1];

              double ref_x_prev = previous_path_x[prev_size-2];
              double ref_y_prev = previous_path_y[prev_size-2];
              ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

              //Use the two points that make the path tangent to the previous path's end point
              ptsx.push_back(ref_x_prev);
              ptsx.push_back(ref_x);

              ptsy.push_back(ref_y_prev);
              ptsy.push_back(ref_y);
              
            }

            double next_s = target.s;
            double next_lane = target.l;
            double next_d = target.d;
            STATE next_state = target.state;
            double next_v = target.v;
            double next_a = target.a;
            ref_vel = next_v;
            ref_a = next_a;            //get target point after horizon time period
            // for(int i = 0; i< trajectory.size(); i++)
            // {
              // vector<double> next_wp = //trajectory[i];
              // assert(next_wp.size() == 2);
              // next_s = next_wp[0];
              // next_lane = next_wp[1];
            vector<double> next_wp = getXY(next_s,next_d,map_waypoints_s,map_waypoints_x,map_waypoints_y); 
            ptsx.push_back(next_wp[0]);
            ptsy.push_back(next_wp[1]); 
            cout<<"next_wp=> x: "<<next_wp[0]<<"\ty: "<<next_wp[1]<<"\n";
            // }

            //In Frenet add evenly 30m spaced points ahead of the starting reference
            // vector<double> next_wp0 = getXY(next_s+30,next_d,map_waypoints_s,map_waypoints_x,map_waypoints_y);
            // vector<double> next_wp1 = getXY(next_s+60,(2+4*next_lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
            // vector<double> next_wp2 = getXY(next_s+90,(2+4*next_lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);

            // ptsx.push_back(next_wp0[0]);
            // ptsx.push_back(next_wp1[0]);
            // ptsx.push_back(next_wp2[0]);

            // ptsy.push_back(next_wp0[1]);
            // ptsy.push_back(next_wp1[1]);
            // ptsy.push_back(next_wp2[1]);
          
            
            for(int i=0; i< ptsx.size(); i++)
            {
              //shift car reference angle to 0 degree
              double shift_x = ptsx[i]-ref_x;
              double shift_y = ptsy[i]-ref_y;

              ptsx[i] = (shift_x * cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
              ptsy[i] = (shift_x * sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
              
            }
            cout<<"\nptsx size: "<<ptsx.size()<<"\tptsy size: "<<ptsy.size()<<"\n";
            for(int i=0; i< ptsx.size(); i++)
            {
              cout<<"["<<i<<"]: "<<ptsx[i]<<" , "<<ptsy[i]<<"\n";
            }
            cout<<"\n";
            //create a spline
            tk::spline s;

            //set (x,y) points to the spline
            s.set_points(ptsx,ptsy);
            
            //Define the actual (x,y) points  we will use in the future
          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

            //Start with all of the previous path points from last time
            for(int i=0; i<previous_path_x.size();i++)
            {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }

            //Calculate how to break up spline points so that we travel at our desired reference velocity
            double target_x = next_wp[0]-car_x;
            // cout<<"\ntarget_x: "<<target_x<<"\n";
            double target_y = s(target_x);
            // cout<<"\ntarget_y: "<<target_y<<"\n";
            double target_dist = sqrt(target_x*target_x + target_y*target_y);

            double x_add_on = 0.0;

            //fill up the rest of our path planner after filling in with previous points, here we will always output 50 points
            cout<<"\nref_vel ==> "<<ref_vel<<"\n";

            double N = (target_dist/(DT*ref_vel/2.24));

            for(int i=1; i<=50 - previous_path_x.size(); i++)
            {

              double x_point = x_add_on+(target_x)/N;
              double y_point = s(x_point);

              x_add_on = x_point;

              double x_ref = x_point;
              double y_ref = y_point;

              //rotate back to normal after rotating it earlier
              x_point = (x_ref * cos(ref_yaw) - y_ref*sin(ref_yaw));
              y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

              x_point += ref_x;
              y_point += ref_y;

              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
            }

            json msgJson;
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
