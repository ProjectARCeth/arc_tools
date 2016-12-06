#include "arc_tools/state_and_path_publisher.hpp"

using namespace arc_tools;

StateAndPathPublisher::StateAndPathPublisher(){
  //Initialising path_array.
  array_position_ = 0;
  path_vector_.clear();
  path_.header.frame_id = "path";
}

void StateAndPathPublisher::createPublisher(ros::NodeHandle* node){
  //Initialising publishing_nodes.
  pub_state_ = node->advertise<arc_msgs::State>("/state", 20);
  pub_path_ = node->advertise<nav_msgs::Path>("/path", 20);
}

void StateAndPathPublisher::publishWithQuaternion(Eigen::Vector3d position, Eigen::Vector4d quat, 
                           Eigen::Vector3d lin_vel, Eigen::Vector3d ang_vel, bool stop){
  //Updating class variables.
  array_position_ += 1;
  //Creating msgs.
  state_.pose.pose.position.x = position(0);
  state_.pose.pose.position.y = position(1);
  state_.pose.pose.position.z = position(2);
  state_.pose.pose.orientation.x = quat(0);
  state_.pose.pose.orientation.y = quat(1);
  state_.pose.pose.orientation.z = quat(2);
  state_.pose.pose.orientation.w = quat(3);
  state_.pose_diff.twist.linear.x = lin_vel(0);
  state_.pose_diff.twist.linear.y = lin_vel(1);
  state_.pose_diff.twist.linear.z = lin_vel(2);
  state_.pose_diff.twist.angular.x = ang_vel(0);
  state_.pose_diff.twist.angular.y = ang_vel(1);
  state_.pose_diff.twist.angular.z = ang_vel(2);
  state_.current_arrayposition = array_position_;
  state_.stop = stop;
  path_.poses.push_back(state_.pose);
  //Publishen.
  pub_state_.publish(state_);
  pub_path_.publish(path_);
}

void StateAndPathPublisher::publishWithEuler(Eigen::Vector3d position, Eigen::Vector3d euler, 
                      Eigen::Vector3d lin_vel, Eigen::Vector3d ang_vel, bool stop){
  //Transforming euler to quaternion.
  Eigen::Vector4d quat = transformQuaternionEulerVector(euler);
  //Updating state and publishing.
  publishWithQuaternion(position, quat, lin_vel, ang_vel, stop);
}

