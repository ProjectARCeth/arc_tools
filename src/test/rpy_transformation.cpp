#include <cmath>

#include "arc_tools/coordinate_transform.hpp"

int main(int argc, char** argv){
	ros::init(argc, argv, "attitudefilter");
	//Defining angles [rad].
	double roll = M_PI;
	double pitch = M_PI/2;
	double yaw = 0;
	Eigen::Vector3d euler;
	euler(0) = roll; euler(1) = pitch; euler(2) = yaw;
	//Trafo Matrix.
	// Eigen::Matrix3d Trafo = arc_tools::getRotationMatrix(euler);
	Eigen::Matrix3d Trafo = getRotationMatrix(euler);
	//Cartesian Coords global.
	double x_global = 1.0;
	double y_global = 0.0;
	double z_global = 0.0;
	Eigen::Vector3d cartesian_global;
	cartesian_global << x_global, y_global, z_global;
	//Cartesian Coords local.
	Eigen::Vector3d cartesian_local;
	cartesian_local = Trafo * cartesian_global; 
	//Printing.
	ROS_INFO("Roll: %f, Pitch: %f, Yaw: %f", euler(0), euler(1), euler(2));
	ROS_INFO("Global Coords: %f, %f, %f", cartesian_global(0), cartesian_global(1), cartesian_global(2));
	ROS_INFO("Local Coords: %f, %f, %f", cartesian_local(0), cartesian_local(1), cartesian_local(2));

	return 0;
}
