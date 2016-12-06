#ifndef COORDINATE_TRANSFORM_ARC_TOOLS_HPP
#define COORDINATE_TRANSFORM_ARC_TOOLS_HPP

#include <cmath>
#include <eigen3/Eigen/Eigen>

#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"

// namespace arc_tools{

geometry_msgs::Quaternion transformQuaternionEulerMsg(const Eigen::Vector3d euler);
Eigen::Vector4d transformQuaternionEulerVector(const Eigen::Vector3d euler);
geometry_msgs::Vector3 transformEulerQuaternionMsg(const Eigen::Vector4d quat);
Eigen::Vector3d transformEulerQuaternionVector(const Eigen::Vector4d quat);

Eigen::Matrix3d getRotationMatrix(const Eigen::Vector3d angles);
Eigen::Matrix3d getAngularVelocityTransformationMatrix(const Eigen::Vector3d angles);

Eigen::Vector3d transformVectorMessageToEigen(const geometry_msgs::Vector3 msg);
Eigen::Vector3d transformPointMessageToEigen(const geometry_msgs::Point msg);
Eigen::Vector4d transformQuatMessageToEigen(const geometry_msgs::Quaternion msg);

// }//namespace arc_tools.

#endif