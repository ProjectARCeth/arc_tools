#ifndef SIGNAL_FILTER_ARC_TOOLS_HPP
#define SIGNAL_FILTER_ARC_TOOLS_HPP

#include "Eigen/Dense"
#include <cmath>
#include <iostream>

namespace arc_tools{

Eigen::VectorXd firstOrderLowPassIIRFilter(Eigen::VectorXd input, Eigen::VectorXd last_output, 
                                          float alpha);
Eigen::VectorXd limitFilter(Eigen::VectorXd input, double limit);

}//namespace arc_tools.

#endif