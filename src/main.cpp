#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "ukf.h"
#include "tools.h"
#include <fstream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::string;
using std::vector;

using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;

  // Create a Kalman Filter instance
  UKF ukf;

  // used to compute the RMSE later
  Tools tools;
  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;

// chris: 
  // for all the lines:
  //   read one line data from file obj_pose-laser-radar-synthetic-input.txt
  //   pack the message according to the type L or R
  //   pack the gt and push into ground_truth vector
  //   call ukf.ProcessMeasurement
  //   push the ukf.x_ into the estimation vector
  //   calculate the RMSE
  string sensor_measurment;
  ifstream DataFile("../data/obj_pose-laser-radar-synthetic-input.txt");
  //ifstream DataFile("../data/radar.txt");
  //ifstream DataFile("../data/laser.txt");

  if (DataFile.is_open())
  {
    while ( getline(DataFile,sensor_measurment) )
    {
      MeasurementPackage meas_package;
      istringstream iss(sensor_measurment);
    	long long timestamp;

    	//reads the sensor type from the current line
    	string sensor_type;
    	iss >> sensor_type;

      //pack the message according to the sensor type
      //LASER: sensor_type px py timestamp
      //RADAR: sensor_type ro theta ro_dot timestamp
      if (sensor_type.compare("L") == 0) 
      {
        meas_package.sensor_type_ = MeasurementPackage::LASER;
        meas_package.raw_measurements_ = VectorXd(2);
        float px;
        float py;
        iss >> px;
        iss >> py;
        meas_package.raw_measurements_ << px, py;
        iss >> timestamp;
        meas_package.timestamp_ = timestamp;
      } 
      else if (sensor_type.compare("R") == 0) 
      {
        meas_package.sensor_type_ = MeasurementPackage::RADAR;
        meas_package.raw_measurements_ = VectorXd(3);
        float ro;
        float theta;
        float ro_dot;
        iss >> ro;
        iss >> theta;
        iss >> ro_dot;
        meas_package.raw_measurements_ << ro,theta, ro_dot;
        iss >> timestamp;
        meas_package.timestamp_ = timestamp;
      }

      //call kalman filter
      ukf.ProcessMeasurement(meas_package);

      //get the ground truth
      float x_gt;
      float y_gt;
      float vx_gt;
      float vy_gt;
      iss >> x_gt;
      iss >> y_gt;
      iss >> vx_gt;
      iss >> vy_gt;
      VectorXd gt_values(4);
      gt_values(0) = x_gt;
      gt_values(1) = y_gt; 
      gt_values(2) = vx_gt;
      gt_values(3) = vy_gt;
      ground_truth.push_back(gt_values); 

      //Push the current estimated x,y positon from the Kalman filter's state vector
      VectorXd estimate(4);

      double p_x = ukf.x_(0);
      double p_y = ukf.x_(1);
      double v1  = ukf.x_(2) * cos(ukf.x_(3));
      double v2  = ukf.x_(2) * sin(ukf.x_(3));

      estimate(0) = p_x;
      estimate(1) = p_y;
      estimate(2) = v1;
      estimate(3) = v2;
      
      estimations.push_back(estimate); 
 
      VectorXd RMSE = tools.CalculateRMSE(estimations, ground_truth);

      //Expect RMSE: 0.09 0.10 0.40 0.30
      // laser RMSE: 0.09 0.10 0.55 0.28
      // radar RMSE: 0.15 0.25 0.22 0.27
      //  both RMSE: 0.06 0.08 0.28 0.21, init with radar data RMSE: 0.06 0.08 0.15 0.22
      //  EKF  RMSE: 0.10 0.09 0.45 0.47
      cout << "RMSE: " << RMSE << endl;

    }
    DataFile.close();
  }

  h.onMessage([&ukf,&tools,&estimations,&ground_truth]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          string sensor_measurement = j[1]["sensor_measurement"];
          
          MeasurementPackage meas_package;
          std::istringstream iss(sensor_measurement);
          
          long long timestamp;

          // reads first element from the current line
          string sensor_type;
          iss >> sensor_type;

          if (sensor_type.compare("L") == 0) {
            meas_package.sensor_type_ = MeasurementPackage::LASER;
            meas_package.raw_measurements_ = VectorXd(2);
            float px;
            float py;
            iss >> px;
            iss >> py;
            meas_package.raw_measurements_ << px, py;
            iss >> timestamp;
            meas_package.timestamp_ = timestamp;
          } else if (sensor_type.compare("R") == 0) {
            meas_package.sensor_type_ = MeasurementPackage::RADAR;
            meas_package.raw_measurements_ = VectorXd(3);
            float ro;
            float theta;
            float ro_dot;
            iss >> ro;
            iss >> theta;
            iss >> ro_dot;
            meas_package.raw_measurements_ << ro,theta, ro_dot;
            iss >> timestamp;
            meas_package.timestamp_ = timestamp;
          }

          float x_gt;
          float y_gt;
          float vx_gt;
          float vy_gt;
          iss >> x_gt;
          iss >> y_gt;
          iss >> vx_gt;
          iss >> vy_gt;

          VectorXd gt_values(4);
          gt_values(0) = x_gt;
          gt_values(1) = y_gt; 
          gt_values(2) = vx_gt;
          gt_values(3) = vy_gt;
          ground_truth.push_back(gt_values);
          
          // Call ProcessMeasurement(meas_package) for Kalman filter
          ukf.ProcessMeasurement(meas_package);       

          // Push the current estimated x,y positon from the Kalman filter's 
          //   state vector

          VectorXd estimate(4);

          double p_x = ukf.x_(0);
          double p_y = ukf.x_(1);
          double v   = ukf.x_(2);
          double yaw = ukf.x_(3);

          double v1 = cos(yaw)*v;
          double v2 = sin(yaw)*v;

          estimate(0) = p_x;
          estimate(1) = p_y;
          estimate(2) = v1;
          estimate(3) = v2;
        
          estimations.push_back(estimate);

          VectorXd RMSE = tools.CalculateRMSE(estimations, ground_truth);

          json msgJson;
          msgJson["estimate_x"] = p_x;
          msgJson["estimate_y"] = p_y;
          msgJson["rmse_x"] =  RMSE(0);
          msgJson["rmse_y"] =  RMSE(1);
          msgJson["rmse_vx"] = RMSE(2);
          msgJson["rmse_vy"] = RMSE(3);
          auto msg = "42[\"estimate_marker\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

        }  // end "telemetry" if

      } else {
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if

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