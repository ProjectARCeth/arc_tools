#include "arc_tools/coordinate_transform.hpp"

namespace arc_tools {

geometry_msgs::Quaternion transformQuaternionEulerMsg(const geometry_msgs::Vector3 euler){
  geometry_msgs::Quaternion quat;
  //Transformation.
  float roll = euler.x;
  float pitch = euler.y;
  float yaw = euler.z; 
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

Eigen::Vector4d transformQuaternionEulerVector(const Eigen::Vector3d euler){
  Eigen::Vector4d quat;
  //Transformation.
  float roll = euler(0);
  float pitch = euler(1);
  float yaw = euler(2); 
  double t0 = cos(yaw * 0.5f);
  double t1 = sin(yaw * 0.5f);
  double t2 = cos(roll * 0.5f);
  double t3 = sin(roll * 0.5f);
  double t4 = cos(pitch * 0.5f);
  double t5 = sin(pitch * 0.5f);
  quat(3) = t0 * t2 * t4 + t1 * t3 * t5;
  quat(0) = t0 * t3 * t4 - t1 * t2 * t5;
  quat(1) = t0 * t2 * t5 + t1 * t3 * t4;
  quat(2) = t1 * t2 * t4 - t0 * t3 * t5;
  return quat;
}

geometry_msgs::Vector3 transformEulerQuaternionMsg(const Eigen::Vector4d quat){
  geometry_msgs::Vector3 euler;
  //Transformation.
	double ysqr = quat(1) * quat(1);

	// roll (x-axis rotation)
	double t0 = +2.0 * (quat(3) * quat(0) + quat(1) * quat(2));
	double t1 = +1.0 - 2.0 * (quat(0) * quat(0) + ysqr);
	euler.x = std::atan2(t0, t1);

	// pitch (y-axis rotation)
	double t2 = +2.0 * (quat(3) * quat(1) - quat(2) * quat(0));
	t2 = t2 > 1.0 ? 1.0 : t2;
	t2 = t2 < -1.0 ? -1.0 : t2;
	euler.y = std::asin(t2);

	// yaw (z-axis rotation)
	double t3 = +2.0 * (quat(3) * quat(2) + quat(0) * quat(1));
	double t4 = +1.0 - 2.0 * (ysqr + quat(2) * quat(2));  
	euler.z = std::atan2(t3, t4);
  return euler;
}

Eigen::Vector3d transformEulerQuaternionVector(const Eigen::Vector4d quat){
  Eigen::Vector3d euler;
  //Transformation.
	double ysqr = quat(1) * quat(1);

	// roll (x-axis rotation)
	double t0 = +2.0 * (quat(3) * quat(0) + quat(1) * quat(2));
	double t1 = +1.0 - 2.0 * (quat(0) * quat(0) + ysqr);
	euler(0) = std::atan2(t0, t1);

	// pitch (y-axis rotation)
	double t2 = +2.0 * (quat(3) * quat(1) - quat(2) * quat(0));
	t2 = t2 > 1.0 ? 1.0 : t2;
	t2 = t2 < -1.0 ? -1.0 : t2;
	euler(1) = std::asin(t2);

	// yaw (z-axis rotation)
	double t3 = +2.0 * (quat(3) * quat(2) + quat(0) * quat(1));
	double t4 = +1.0 - 2.0 * (ysqr + quat(2) * quat(2));  
	euler(2) = std::atan2(t3, t4);
  return euler;
}

Eigen::Matrix3d getRotationMatrix(const Eigen::Vector3d angles){
  double phi = angles(0);
  double theta = angles(1);
  double psi = angles(2);
  //Rotation matrix.
  Eigen::Matrix3d rotation_matrix;
  rotation_matrix(0,0) = cos(theta)*cos(psi);
  rotation_matrix(0,1) = -cos(theta)*sin(psi); 
  rotation_matrix(0,2) = sin(theta); 
  rotation_matrix(1,0) = sin(phi)*sin(theta)*cos(psi) + cos(phi)*sin(psi);
  rotation_matrix(1,1) = -sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi); 
  rotation_matrix(1,2) = -sin(phi)*cos(theta); 
  rotation_matrix(2,0) = -cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi);
  rotation_matrix(2,1) = cos(phi)*sin(theta)*sin(psi) + sin(phi)*cos(psi); 
  rotation_matrix(2,2) = cos(phi)*cos(theta); 
  return rotation_matrix;
}

Eigen::Matrix3d getAngularVelocityTransformationMatrix(const Eigen::Vector3d angles){
  Eigen::Matrix<double,3,3> Trafomatrix;
  double roll = angles(0);
  double pitch = angles(1);
  double yaw = angles(2);
  Trafomatrix(0,0) = 1.0; Trafomatrix(0,1) = sin(roll)*tan(pitch); Trafomatrix(0,2) = cos(roll)*tan(pitch);
  Trafomatrix(1,0) = 0.0; Trafomatrix(1,1) = cos(roll); Trafomatrix(1,2) = -sin(roll);
  Trafomatrix(2,0) = 0.0; Trafomatrix(2,1) = sin(roll)/cos(pitch); Trafomatrix(2,2) = cos(roll)/cos(pitch);
  return Trafomatrix;
}

Eigen::Vector3d transformVectorMessageToEigen(const geometry_msgs::Vector3 msg){
  Eigen::Vector3d eigen_vector;
  eigen_vector(0) = msg.x;
  eigen_vector(1) = msg.y;
  eigen_vector(2) = msg.z;
  return eigen_vector;
}

Eigen::Vector3d transformPointMessageToEigen(const geometry_msgs::Point msg){
  Eigen::Vector3d eigen_vector;
  eigen_vector(0) = msg.x;
  eigen_vector(1) = msg.y;
  eigen_vector(2) = msg.z;
  return eigen_vector;
}

geometry_msgs::Point transformEigenToPointMessage(const Eigen::Vector3d msg){
  geometry_msgs::Point point;
  point.x=msg(0);
  point.y=msg(1);
  point.z=msg(2);

  return point;
}
Eigen::Vector4d transformQuatMessageToEigen(const geometry_msgs::Quaternion msg){
  Eigen::Vector4d eigen_vector;
  eigen_vector(0) = msg.x;
  eigen_vector(1) = msg.y;
  eigen_vector(2) = msg.z;
  eigen_vector(3) = msg.w;
  return eigen_vector;
} 
}//namespace arc_tools.
