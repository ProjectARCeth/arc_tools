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
  double alpha = angles(0);
  double beta = angles(1);
  double gamma = angles(2);
  //Rotation matrix.
  Eigen::Matrix3d rotation_matrix;
  Eigen::Matrix3d rotation_z_axis;
  Eigen::Matrix3d rotation_xnew_axis;
  Eigen::Matrix3d rotation_znewnew_axis;
  rotation_z_axis<<	cos(alpha),	sin(alpha),		0,
				-sin(alpha),cos(alpha),		0,
				0,		0,			1;

  rotation_xnew_axis<<	1,		0,			0,
				0,		cos(beta),		sin(beta),
				0,		-sin(beta),		cos(beta);

  rotation_znewnew_axis<<	cos(gamma),	sin(gamma),		0,
				-sin(gamma),cos(gamma),		0,
				0,		0,			1;

	rotation_matrix= rotation_z_axis * rotation_xnew_axis * rotation_znewnew_axis;
  Eigen::Matrix3d rotation_matri;
  rotation_matri(0,0) = cos(beta)*cos(gamma);
  rotation_matri(0,1) = -cos(beta)*sin(gamma); 
  rotation_matri(0,2) = sin(beta); 
  rotation_matri(1,0) = sin(alpha)*sin(beta)*cos(gamma) + cos(alpha)*sin(gamma);
  rotation_matri(1,1) = -sin(alpha)*sin(beta)*sin(gamma) + cos(alpha)*cos(gamma); 
  rotation_matri(1,2) = -sin(alpha)*cos(beta); 
  rotation_matri(2,0) = -cos(alpha)*sin(beta)*cos(gamma) + sin(alpha)*sin(gamma);
  rotation_matri(2,1) = cos(alpha)*sin(beta)*sin(gamma) + sin(alpha)*cos(gamma); 
  rotation_matri(2,2) = cos(alpha)*cos(beta);
  std::cout<<rotation_matri<<std::endl<<std::endl; 
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

geometry_msgs::Point globalToLocal(const geometry_msgs::Point global_koordinate, arc_msgs::State new_frame_origin)
{
  //Translatation
  std::cout<<"global: "<<global_koordinate.x<<" "<<global_koordinate.y<<std::endl;
  Eigen::Vector3d glob=arc_tools::transformPointMessageToEigen(global_koordinate);
  Eigen::Vector3d stat= arc_tools::transformPointMessageToEigen(new_frame_origin.pose.pose.position);
//std::cout<<stat(0)<<" "<<stat(1)<<" "<<stat(2)<<std::endl;
  Eigen::Vector3d temp=glob-stat;
//std::cout<<temp(0)<<" "<<temp(1)<<" "<<temp(2)<<std::endl;
  //Rotation
  Eigen::Vector4d quat=transformQuatMessageToEigen(new_frame_origin.pose.pose.orientation);
  Eigen::Vector3d euler=transformEulerQuaternionVector(quat);
  Eigen::Matrix3d R=getRotationMatrix(euler);
  Eigen::Matrix3d T=R.transpose();
//std::cout<<R(0,0)<<" "<<R(0,1)<<" "<<R(0,2)<<std::endl<<" "<<R(1,0)<<" "<<R(1,1)<<" "<<R(1,2)<<std::endl<<" "<<R(2,0)<<" "<<R(2,1)<<" "<<R(2,2)<<std::endl;
  std::cout<<"state: "<<new_frame_origin.pose.pose.position.x<<" "<<new_frame_origin.pose.pose.position.y<<" "<<euler(2)<<std::endl; 
  Eigen::Vector3d local=T*temp;
//std::cout<<local(0)<<" "<<local(1)<<" "<<local(2)<<std::endl;
  geometry_msgs::Point local_msg=transformEigenToPointMessage(local);
  std::cout<<"local: "<<local_msg.x<<" "<<local_msg.y<<" "<<std::endl;
  return local_msg;
}

arc_msgs::State generate2DState(const float x, const float y, const float alpha )
{
  arc_msgs::State state;
  state.pose.pose.position.x=x;
  state.pose.pose.position.y=y;
  geometry_msgs::Vector3 eu;
  eu.x=0;
  eu.y=0;
  eu.z=alpha;
  geometry_msgs::Quaternion quat;
  quat=arc_tools::transformQuaternionEulerMsg(eu);
  state.pose.pose.orientation=quat;
  return state;

}
}//namespace arc_tools.
