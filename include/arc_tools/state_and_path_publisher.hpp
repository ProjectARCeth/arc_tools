#ifndef STATE_AND_PATH_PUBLISHER_ARC_TOOLS_HPP
#define STATE_AND_PATH_PUBLISHER_ARC_TOOLS_HPP

#include "arc_msgs/State.h"
#include "arc_tools/coordinate_transform.hpp"

#include "ros/ros.h"
#include "Eigen/Dense"
#include <iostream>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Path.h"

namespace arc_tools{

class StateAndPathPublisher {
public:
	StateAndPathPublisher();
	void createPublisher(ros::NodeHandle* node);
	//State (x,y,z  phi,theta,psi, x_dot,y_dot,z_dot, phi_dot,theta_dot,psi_dot, ...).
	void publish(Eigen::VectorXd x, bool stop);

private:
	void updateStateVariables(Eigen::VectorXd x, bool stop);
    ros::Publisher pub_state_;
    ros::Publisher pub_path_;
	Eigen::Matrix<double,12,1> x_;
	bool stop_;
	int array_position_;
	geometry_msgs::Quaternion quat_;
	arc_msgs::State state_;
	std::vector<geometry_msgs::PoseStamped> path_vector_;
	nav_msgs::Path path_;
};
}//namespace arc_tools.

#endif