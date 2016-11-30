#include "arc_tools/state_and_path_publisher.hpp"

namespace arc_tools{

StateAndPathPublisher::StateAndPathPublisher(){
  //Initialising path_array.
  array_position_ = 0;
  path_vector_.clear();
  path_.header.frame_id = "header";
}

void StateAndPathPublisher::createPublisher(ros::NodeHandle* node){
  //Initialising publishing_nodes.
  pub_state_ = node->advertise<arc_msgs::State>("/state", 20);
  pub_path_ = node->advertise<nav_msgs::Path>("/path", 20);
}

void StateAndPathPublisher::publish(Eigen::VectorXd x, bool stop){
  //Updating class variables.
  updateStateVariables(x, stop);
  //Creating msgs.
  state_.pose.pose.position.x = x_(0,0);
  state_.pose.pose.position.y = x_(1,0);
  state_.pose.pose.position.z = x_(2,0);
  state_.pose.pose.orientation = transformQuaternionEuler(x_.block<3,1>(3,0));
  state_.pose_diff.twist.linear.x = x_(6,0);
  state_.pose_diff.twist.linear.y = x_(7,0);
  state_.pose_diff.twist.linear.z = x_(8,0);
  state_.pose_diff.twist.angular.x = x_(9,0);
  state_.pose_diff.twist.angular.y = x_(10,0);
  state_.pose_diff.twist.angular.z = x_(11,0);
  state_.current_arrayposition = array_position_;
  state_.stop = stop_;
  path_.poses.push_back(state_.pose);
  //Publishen.
  pub_state_.publish(state_);
  pub_path_.publish(path_);
}

void StateAndPathPublisher::updateStateVariables(Eigen::VectorXd x, bool stop){
  //Updating state (x,y,z  phi,theta,psi, x_dot,y_dot,z_dot, phi_dot,theta_dot,psi_dot).
  x_.segment<12>(0) = x.segment<12>(0);
  //Updating array_position and stop.
  array_position_ += 1;
  stop_ = stop;
}
}


