#ifndef COORDINATE_TRANSFORM_ARC_TOOLS_HPP
#define COORDINATE_TRANSFORM_ARC_TOOLS_HPP

#include <cmath>
#include <eigen3/Eigen/Eigen>

#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include "arc_msgs/State.h"

namespace arc_tools {

geometry_msgs::Quaternion transformQuaternionEulerMsg(const geometry_msgs::Vector3 euler);
Eigen::Vector4d transformQuaternionEulerVector(const Eigen::Vector3d euler);
geometry_msgs::Vector3 transformEulerQuaternionMsg(const Eigen::Vector4d quat);
Eigen::Vector3d transformEulerQuaternionVector(const Eigen::Vector4d quat);

Eigen::Matrix3d getRotationMatrix(const Eigen::Vector3d angles);
Eigen::Matrix3d getAngularVelocityTransformationMatrix(const Eigen::Vector3d angles);

Eigen::Vector3d transformVectorMessageToEigen(const geometry_msgs::Vector3 msg);
Eigen::Vector3d transformPointMessageToEigen(const geometry_msgs::Point msg);
Eigen::Vector4d transformQuatMessageToEigen(const geometry_msgs::Quaternion msg);
geometry_msgs::Point transformEigenToPointMessage(const Eigen::Vector3d msg);

geometry_msgs::Point globalToLocal(geometry_msgs::Point global_koordinate,arc_msgs::State new_frame_origin);
}//namespace arc_tools.

#endif
