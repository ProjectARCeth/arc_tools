#include "arc_tools/signal_filter.hpp"

// using namespace arc_tools;

Eigen::VectorXd firstOrderLowPassIIRFilter(const Eigen::VectorXd input, const Eigen::VectorXd last_output, 
                                          float alpha){
  //Concurring dimensions?.
  if (input.size() != last_output.size()){std::cout << "dimensions dont fit in filter !";}
  //y[n] = alpha*y[n-1] + (1-alpha)*u[n].
  Eigen::VectorXd current_output = alpha*last_output + (1-alpha)*input;
  return current_output;
}

Eigen::VectorXd limitFilter(Eigen::VectorXd input, double limit){
	int length = input.size();
	for (int i = 0; i < length; ++i)
	{
		if (abs(input(i)<limit)){input(i) = 0.0;}
	}
  return input;
}

