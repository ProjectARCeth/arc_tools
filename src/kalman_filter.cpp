#include "arc_tools/kalman_filter.hpp"

namespace arc_tools{

KalmanFilter::KalmanFilter(){
  initialized_ = false;
}

void KalmanFilter::init(const Eigen::VectorXd& x0){
  dimension_ = A_.rows();
  //Initialsing state Vector, Input and Covariance Matrices.
  x_ = x0;
  xbar_ = Eigen::VectorXd::Zero(dimension_);
  u_ = Eigen::VectorXd::Zero(B_.cols());
  P_ = Eigen::MatrixXd::Zero(dimension_, dimension_);
  Pbar_ = Eigen::MatrixXd::Zero(dimension_, dimension_);
  I_ = Eigen::MatrixXd::Identity(dimension_, dimension_);
  initialized_ = true;
  //Initialising Time.
  time_.start();
}

void KalmanFilter::kalmanCore(const Eigen::VectorXd& z){
  //Filter already initialised ?
  if (initialized_ == false && timestep_ < 0) ROS_INFO("Init Kalman first !");
  //Core Kalman Algorithm.
  xbar_ = A_ * x_ + B_ * u_;
  Pbar_ = A_ * P_ * A_.transpose() + Q_;
  K_ = Pbar_ * H_.transpose() * (H_ * Pbar_ * H_.transpose() + R_).inverse();
  x_ = xbar_ + K_ * (z - H_ * xbar_);
  P_ = (I_ - K_ * H_) * Pbar_;
}

// void KalmanFilterOrientation::initWithErrors(const Eigen::VectorXd& x0,
//         const double error_state_euler_dot, const double error_state_linear_acceleration,
//         const double error_measurement_gyro, const double error_measurement_linear_accelerometer){
//   //Initialising state matrices.
//   A_ = Eigen::MatrixXd::Identity(15,15);
//   A_.block<9,9>(0,6) = Eigen::MatrixXd::Identity(9,9) * timestep_;
//   // A_.block<6,6>(9,9) = Eigen::MatrixXd::Zero(6,6);
//   B_ = Eigen::MatrixXd::Zero(15,15);
//   H_ = Eigen::MatrixXd::Zero(6,15);
//   //Initialising error matrices.
//   Q_ = Eigen::MatrixXd::Zero(15,15);
//   Q_.block<3,3>(9,9) = Eigen::MatrixXd::Identity(3,3) * error_state_euler_dot;
//   Q_.block<3,3>(12,12) = Eigen::MatrixXd::Identity(3,3) * error_state_linear_acceleration;
//   R_ = Eigen::MatrixXd::Identity(6,6);
//   R_.block<3,3>(0,0) = Eigen::MatrixXd::Identity(3,3) * error_measurement_linear_accelerometer;
//   R_.block<3,3>(3,3) = Eigen::MatrixXd::Identity(3,3) * error_measurement_gyro;
//   //Initialising kalman Gain.
//   K_ = Eigen::MatrixXd::Zero(15,6);
//   init(x0);
// }

// void KalmanFilterOrientation::updateMatrices(){
//   //Updating A
//   A_.block<9,9>(0,6) = Eigen::MatrixXd::Identity(9,9) * timestep_;
//   //Updating matrix H
//   angles_.segment<3>(0) = x_.segment<3>(3);
//   Eigen::Matrix<double,3,3>RotationMatrix = getRotationMatrix(angles_);
//   //Linear Acceleration Vector is transformed from body into global frame.
//   H_.block<3,3>(0,12) = RotationMatrix;
//   //Angular Velocity Vector is transformed from body into global frame and then 
//   //angular velocities around axes into euler velocities 
//   //(https://www.princeton.edu/~stengel/MAE331Lecture9.pdf).
//   Eigen::Matrix<double,3,3> Trafomatrix;
//   Trafomatrix(0,0) = 1.0; Trafomatrix(0,2) = sin(x_(3))*tan(x_(4)); Trafomatrix(0,2) = cos(x_(3))*tan(x_(4));
//   Trafomatrix(1,0) = 0.0; Trafomatrix(1,1) = cos(x_(3)); Trafomatrix(1,2) = -sin(x_(3));
//   Trafomatrix(2,0) = 0.0; Trafomatrix(2,1) = sin(x_(3))/cos(x_(4)); Trafomatrix(2,2) = cos(x_(3))/cos(x_(4));
//   H_.block<3,3>(3,9) = Trafomatrix*RotationMatrix;
//   //Gravitation vector in global frame alwayse in -z-direction.
//   Eigen::Vector3d direction_g;
//   direction_g(0) = 0.0; direction_g(1) = 0.0; direction_g(2) = -gravity_constant;
//   gravity_orientation_ = RotationMatrix * direction_g;
// } 

// bool KalmanFilterOrientation::update(const sensor_msgs::Imu::ConstPtr & imu_data){
//   //Time Interval.
//   timestep_ = time_.getTimestep();
//   //Update matrices.
//   updateMatrices();
//   //Getting imu-measurements.
//   current_measurements_(0) = imu_data->linear_acceleration.x + gravity_orientation_(0,0);
//   current_measurements_(1) = imu_data->linear_acceleration.y + gravity_orientation_(1,0);
//   current_measurements_(2) = imu_data->linear_acceleration.z + gravity_orientation_(2,0);
//   current_measurements_(3) = imu_data->angular_velocity.x;
//   current_measurements_(4) = imu_data->angular_velocity.y;
//   current_measurements_(5) = imu_data->angular_velocity.z;
//   //Filtering high frequencies.
//   current_measurements_= firstOrderLowPassIIRFilter(current_measurements_,
//                                                     last_measurements_, exp(-timestep_/0.5));
//   current_measurements_= limitFilter(current_measurements_,0.15);
//   last_measurements_ = current_measurements_;

//   //Test with self-made Input.
//   current_measurements_ = Eigen::VectorXd::Zero(6);
//   current_measurements_(0) = 0.005;
//   // H_.block<3,3>(0,12) = Eigen::MatrixXd::Identity(3,3);
//   // H_.block<3,3>(3,9) = Eigen::MatrixXd::Identity(3,3);
//   //Updating using simple ForwardIntegration.
//   simpleForwardIntegration(current_measurements_);

//   //Kalman core algorithm.
//   // kalmanCore(current_measurements_);
//   return true;
// }
}//namespace arc_tools.