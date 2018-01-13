#include <fstream>
#include <iostream>
#include <sstream>
#include <math.h>
#include "FusionEKF.h"
#include "tools.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

int main()
{
  // Create a Kalman Filter instance
  FusionEKF fusionEKF;

  // used to compute the RMSE later
  Tools tools;
  
  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;
  MeasurementPackage meas_package;
  long long timestamp;
  string sensor_type;
  float px;
  float py;
  float ro;
  float theta;
  float ro_dot;
  float x_gt;
  float y_gt;
  float vx_gt;
  float vy_gt;
  double p_x;
  double p_y;
  double v1;
  double v2;

  string in_file_name_ = "../obj_pose-laser-radar-synthetic-input.txt";
	ifstream in_file(in_file_name_.c_str(),std::ifstream::in);
	
	if (in_file.fail()) {
		cout << "main() - File open() failed!!!" << endl;
	}
	else if (!in_file.is_open()) {
		cout << "main() - Cannot open input file: " << in_file_name_ << endl;
	}
  else {
    string line;
    int line_number = 1;
    
    ofstream out_file;
    out_file.open("estimates.txt");
    
		while (getline(in_file, line)) {
      istringstream iss(line);
      iss >> sensor_type;

      if (sensor_type.compare("L") == 0) {
        meas_package.sensor_type_ = MeasurementPackage::LASER;
        meas_package.raw_measurements_ = VectorXd(2);
        iss >> px;
        iss >> py;
        meas_package.raw_measurements_ << px, py;
        iss >> timestamp;
        meas_package.timestamp_ = timestamp;
      } 
      else if (sensor_type.compare("R") == 0) {
        meas_package.sensor_type_ = MeasurementPackage::RADAR;
        meas_package.raw_measurements_ = VectorXd(3);
        iss >> ro;
        iss >> theta;
        iss >> ro_dot;
        meas_package.raw_measurements_ << ro,theta, ro_dot;
        iss >> timestamp;
        meas_package.timestamp_ = timestamp;
      }
      
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
        
      //Call ProcessMeasurment(meas_package) for Kalman filter
      fusionEKF.ProcessMeasurement(meas_package);    	  

      //Push the current estimated x,y position from the Kalman filter's state vector
      VectorXd estimate(4);

      p_x = fusionEKF.ekf_.x_(0);
      p_y = fusionEKF.ekf_.x_(1);
      v1  = fusionEKF.ekf_.x_(2);
      v2 = fusionEKF.ekf_.x_(3);

      estimate(0) = p_x;
      estimate(1) = p_y;
      estimate(2) = v1;
      estimate(3) = v2;
      
      estimations.push_back(estimate);

      VectorXd RMSE = tools.CalculateRMSE(estimations, ground_truth);

      //  rmse_x, rmse_y, rmse_vx, and rmse_vy should be less than or equal to: 
      // [.11, .11, 0.52, 0.52]
      cout << "Result for observation: " << line_number << endl;
      cout << "estimate_x: " << p_x << endl;
      cout << "estimate_y: " << p_y << endl;
      cout << "rmse_x: " << RMSE(0) << endl;
      cout << "rmse_y: " << RMSE(1) << endl;
      cout << "rmse_vx: " << RMSE(2) << endl;
      cout << "rmse_vy: " << RMSE(3) << endl << endl;
      
      out_file << p_x << "\t" << p_y << "\t" << RMSE(0) << "\t" << RMSE(1) << "\t" << RMSE(2) << "\t" << RMSE(3) << endl;
      
      line_number += 1;
    }
    if(out_file.is_open()) {
      out_file.close();
    }
  }

  if(in_file.is_open()) {
		in_file.close();
	}
  
	return 0;
}