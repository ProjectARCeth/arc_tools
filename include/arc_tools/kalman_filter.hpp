#ifndef KALMAN_FILTER_ARC_HEADER_HPP
#define KALMAN_FILTER_ARC_HEADER_HPP

#include "arc_tools/coordinate_transform.hpp"
#include "arc_tools/timing.hpp"

#include "ros/ros.h"
#include "Eigen/Dense"
#include <time.h>
#include <cmath>

#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"

class KalmanFilter {
public:
  KalmanFilter();
  void init(const Eigen::VectorXd& x0,
            const Eigen::MatrixXd& A,
            const Eigen::MatrixXd& B,
            const Eigen::MatrixXd& H,
            const Eigen::MatrixXd& Q,
            const Eigen::MatrixXd& R);
  void init(const Eigen::VectorXd& x0);
  bool kalmanCore(const Eigen::VectorXd& z); 
  //System dynamics A, Input B, observation H, process noise Q, measurement noise R. 
  Eigen::MatrixXd A_, B_, H_, R_, Q_, I_;
  //Covariance P, Kalman Gain K.     
  Eigen::MatrixXd P_, Pbar_, K_; 
  //State x, inputs u and covariance p.            
  Eigen::VectorXd x_, xbar_, u_;             
  int dimension_;
  bool initialized_;
  double timestep_;
  Clock time_;
};

class KalmanFilterOrientation : private KalmanFilter{
public:
  void initWithErrors(const Eigen::VectorXd& x0,
                      const double error_state_euler_dot, const double error_state_linear_acceleration,
                      const double error_measurement_gyro, const double error_measurement_linear_accelerometer);
  bool update(const sensor_msgs::Imu::ConstPtr& imu_data);
  geometry_msgs::Quaternion getOrientation();
  geometry_msgs::Vector3 getAngles();
  geometry_msgs::Point getPoint();
  geometry_msgs::Pose getPose();
  Eigen::VectorXd getState();

private:
  static const double gravityConstant =  9.80665;   
  void updateMatrices();   
  Eigen::Matrix<double,3,1> angles_;
  Eigen::Matrix<double,6,1> current_measurements_;
  Eigen::Matrix<double,3,3> H_block_;
  geometry_msgs::Pose pose_;
};

#endif