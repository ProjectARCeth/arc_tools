#include "arc_tools/coordinate_transform.hpp"

geometry_msgs::Quaternion transformQuaternionEuler(const Eigen::Vector3d euler){

  geometry_msgs::Quaternion quat;

  float roll = euler(0);
  float pitch = euler(1);
  float yaw = euler(2); 

  double t0 = cos(yaw * 0.5f);
  double t1 = sin(yaw * 0.5f);
  double t2 = cos(roll * 0.5f);
  double t3 = sin(roll * 0.5f);
  double t4 = cos(pitch * 0.5f);
  double t5 = sin(pitch * 0.5f);

  quat.w = t0 * t2 * t4 + t1 * t3 * t5;
  quat.x = t0 * t3 * t4 - t1 * t2 * t5;
  quat.y = t0 * t2 * t5 + t1 * t3 * t4;
  quat.z = t1 * t2 * t4 - t0 * t3 * t5;
  
  return quat;
}

geometry_msgs::Vector3 transformEulerQuaternion(const Eigen::Vector4d quat){

  geometry_msgs::Vector3 euler;

  double ysqr = quat(1) * quat(1);
  double t0 = -2.0f * (ysqr + quat(2) * quat(2)) + 1.0f;
  double t1 = +2.0f * (quat(0) * quat(1) - quat(3) * quat(2));
  double t2 = -2.0f * (quat(0) * quat(2) + quat(3) * quat(1));
  double t3 = +2.0f * (quat(1) * quat(2) - quat(3) * quat(0));
  double t4 = -2.0f * (quat(0) * quat(0) + ysqr) + 1.0f;

  t2 = t2 > 1.0f ? 1.0f : t2;
  t2 = t2 < -1.0f ? -1.0f : t2;

  euler.y = std::asin(t2);
  euler.x = std::atan2(t3, t4);
  euler.z = std::atan2(t1, t0);

  return euler;
}

Eigen::Matrix3d getRotationMatrix(const Eigen::Vector3d angles){

  double phi = angles(0);
  double theta = angles(1);
  double psi = angles(2);

  Eigen::Matrix3d rotation_matrix;
  rotation_matrix << cos(-theta)*cos(psi), cos(-theta)*sin(psi), sin(theta),
            sin(phi)*sin(-theta)*cos(phi) - cos(phi)*sin(psi), sin(phi)*sin(-theta)*sin(psi) + cos(phi)*cos(psi), sin(phi)*cos(-theta), 
            cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi), cos(phi)*sin(-theta)*sin(psi) - sin(phi)*cos(psi), cos(phi)*cos(theta);

  return rotation_matrix;
}
