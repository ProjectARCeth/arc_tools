#ifndef COORDINATE_TRANSFORM_ARC_TOOLS_HPP
#define COORDINATE_TRANSFORM_ARC_TOOLS_HPP

#include <cmath>
#include <eigen3/Eigen/Eigen>

#include "ros/ros.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"

// namespace arc_tools{

geometry_msgs::Quaternion transformQuaternionEuler(const Eigen::Vector3d euler);
geometry_msgs::Vector3 transformEulerQuaternion(const Eigen::Vector4d quat);

Eigen::Matrix3d getRotationMatrix(const Eigen::Vector3d angles);
Eigen::Matrix3d getAngularVelocityTransformationMatrix(const Eigen::Vector3d angles);

// }//namespace arc_tools.

#endif