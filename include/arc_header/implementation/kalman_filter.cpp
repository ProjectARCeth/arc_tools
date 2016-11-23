#include "arc_header/kalman_filter.hpp"

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
  I_ = Eigen::MatrixXd::Identity(A_.rows(), A_.cols());
  initialized_ = true;
  //Initialising Time.
  time.start();
}

void KalmanFilter::init(const Eigen::VectorXd& x0,
                        const Eigen::MatrixXd& A,
                        const Eigen::MatrixXd& B,
                        const Eigen::MatrixXd& H,
                        const Eigen::MatrixXd& Q,
                        const Eigen::MatrixXd& R){
  A_ = A;
  B_ = B;
  H_ = H;
  Q_ = Q;
  R_ = R;
  //Initialising Vectors and other class variables.
  init(x0);
}

bool KalmanFilter::kalmanCore(const Eigen::VectorXd& z){
  //Filter already initialised ?
  if (initialized_ == false && timestep_ < 0) return initialized_;
  //Core Kalman Algorithm.
  xbar_ = A_ * x_ + B_ * u_;
  Pbar_ = A_ * P_ * A_.transpose() + Q_;
  K_ = Pbar_ * H_.transpose() * (H_ * Pbar_ * H_.transpose() + R_).inverse();
  x_ = xbar_ + K_ * (z - H_ * xbar_);
  P_ = (I_ - K_ * H_) * Pbar_;
  return true;
}

Eigen::VectorXd KalmanFilter::state(){
  return x_;
}

void KalmanFilterOrientation::initWithErrors(const Eigen::VectorXd& x0,
        const double error_state_euler_dot, const double error_state_linear_acceleration,
        const double error_measurement_gyro, const double error_measurement_linear_accelerometer){
  //Initialising state matrices.
  A_ = Eigen::MatrixXd::Identity(15,15);
  A_.block<9,9>(0,6) = Eigen::MatrixXd::Identity(9,9) * timestep_;
  A_.block<3,3>(12,12) = Eigen::MatrixXd::Zero(3,3);
  B_ = Eigen::MatrixXd::Zero(15,15);
  H_ = Eigen::MatrixXd::Zero(6,15);
  H_.block<3,3>(2,9) = Eigen::MatrixXd::Identity(3,3);
  //Initialising error matrices.
  Q_ = Eigen::MatrixXd::Identity(15,15);
  Q_.block<3,3>(9,9) = Eigen::MatrixXd::Identity(3,3) * error_state_euler_dot;
  Q_.block<3,3>(12,12) = Eigen::MatrixXd::Identity(3,3) * error_state_linear_acceleration;
  R_ = Eigen::MatrixXd::Identity(6,6);
  R_.block<3,3>(0,0) = Eigen::MatrixXd::Identity(3,3) * error_measurement_linear_accelerometer;
  R_.block<3,3>(3,3) = Eigen::MatrixXd::Identity(3,3) * error_measurement_gyro;
  //Initialising kalman Gain.
  K_ = Eigen::MatrixXd::Zero(15,6);
  init(x0);
}

void KalmanFilterOrientation::updateMatrices(){
  //Updating A
  A_.block<9,9>(0,6) = Eigen::MatrixXd::Identity(9,9) * timestep_;
  //Updating matrix H
  angles_.segment<3>(0) = x_.segment<3>(3);
  H_.block<3,3>(0,12) = getRotationMatrix(angles_);
  //ROS_INFO("A(0,6): %f", A_(0, 6));               //Matrix A and C CHECKED.
  //ROS_INFO("%f and %f", H_(0, 12), H_(1,12));     //Rotational matrix CHECKED.
} 

bool KalmanFilterOrientation::update(const sensor_msgs::Imu::ConstPtr & imu_data){
  //Time Interval.
  timestep_ = time.getTimestep();
  ROS_INFO("timestep: %f", timestep_);                       //Time Interval CHECKED.
  //Update matrices.
  updateMatrices();
  //Getting imu-measurements.
  current_measurements_(0) = imu_data->linear_acceleration.x;
  current_measurements_(1) = imu_data->linear_acceleration.y;
  //Compensating external acceleration --> x_ is transformed, therefore g always in z-direction.
  current_measurements_(2) = imu_data->linear_acceleration.z + gravityConstant; 
  current_measurements_(3) = imu_data->angular_velocity.x;
  current_measurements_(4) = imu_data->angular_velocity.y;
  current_measurements_(5) = imu_data->angular_velocity.z;
  //Kalman core algorithm.
  if (!kalmanCore(current_measurements_)){
    ROS_INFO("Please initialize first !");
  }
  return true;
}

geometry_msgs::Quaternion KalmanFilterOrientation::getOrientation(){
  angles_.segment<3>(0) = x_.segment<3>(3);
  return transformQuaternionEuler(angles_);
}

geometry_msgs::Vector3 KalmanFilterOrientation::getAngles(){
  geometry_msgs::Vector3 euler;
  euler.x = x_(3);
  euler.y = x_(4);
  euler.z = x_(5);
  return euler;
}

geometry_msgs::Point KalmanFilterOrientation::getPoint(){
  geometry_msgs::Point point;
  point.x = x_(0);
  point.y = x_(1);
  point.z = x_(2);
  return point;
}

geometry_msgs::Pose KalmanFilterOrientation::getPose(){
  pose_.position = getPoint();
  pose_.orientation = getOrientation();
  return pose_;
}

