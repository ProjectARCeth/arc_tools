#ifndef KALMAN_FILTER_ARC_HEADER_HPP
#define KALMAN_FILTER_ARC_HEADER_HPP

#include "arc_header/coordinate_transform.hpp"
#include "arc_header/timing.hpp"

#include "ros/ros.h"
#include "Eigen/Dense"
#include <time.h>

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
  Eigen::VectorXd state();
  //System dynamics A, Input B, observation H, process noise Q, measurement noise R. 
  Eigen::MatrixXd A_, B_, H_, R_, Q_, I_;
  //Covariance P, Kalman Gain K.     
  Eigen::MatrixXd P_, Pbar_, K_; 
  //State x, inputs u and covariance p.            
  Eigen::VectorXd x_, xbar_, u_;             
  int dimension_;
  bool initialized_;
  double timestep_;
  Clock time;
};

class KalmanFilterOrientation : private KalmanFilter{
public:
  void initWithErrors(const Eigen::VectorXd& x0,
                      const double errorStateEulerDot, const double errorStateLinearAcceleration,
                      const double errorMeasurementGyro, const double errorMeasurementLinearAccelerometer);
  bool update(const sensor_msgs::Imu::ConstPtr& imuData);
  geometry_msgs::Quaternion getOrientation();
  geometry_msgs::Vector3 getAngles();
  geometry_msgs::Point getPoint();
  geometry_msgs::Pose getPose();

private:
  static const double gravityConstant =  9.80665;   
  void updateMatrices();   
  Eigen::Matrix<double,3,1> angles_;
  Eigen::Matrix<double,6,1> currentMeasurements_;
  geometry_msgs::Pose pose_;
};

#endif