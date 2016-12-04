#ifndef KALMAN_FILTER_ARC_TOOLS_HPP
#define KALMAN_FILTER_ARC_TOOLS_HPP

#include "arc_tools/coordinate_transform.hpp"
#include "arc_tools/signal_filter.hpp"
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

namespace arc_tools{

class KalmanFilter {
public:
  KalmanFilter();
  void init(const Eigen::VectorXd& x0);
  void kalmanCore(const Eigen::VectorXd& z); 
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
}//namespace arc_tools.

#endif