#ifndef TIMING_ARC_TOOLS_HPP
#define TIMING_ARC_TOOLS_HPP

namespace arc_tools{

class Clock {

 public:
  Clock() { start(); first_step_ = true;}
  double getTimestep(){
  	if(first_step_){
  		last_time_ = getTime();
      first_step_ = false;
  	}
  	current_time_ = getTime();
  	time_step_ = current_time_ - last_time_;
  	last_time_ = getTime();
  	return time_step_ * kMilisecondsToSeconds;
  }
  void start(){ gettimeofday(&real_time_start_, NULL); }
 private:
  double getTime() { takeTime(); return getRealTime(); }
  double getRealTime() { return real_time_ms_;}
  void takeTime(){
  	//Updating cpu Time
    struct timeval end;
    gettimeofday(&end, NULL);
    long seconds, useconds;
    seconds  = end.tv_sec  - real_time_start_.tv_sec;
    useconds = end.tv_usec - real_time_start_.tv_usec;
    real_time_ms_ = (seconds * kSecondsToMiliseconds +
        useconds * kMicrosecondsToMiliseconds) + 0.5;
  }
  struct timeval real_time_start_;
  double real_time_ms_;
  bool first_step_;
  double last_time_, current_time_, time_step_;
  static const double kSecondsToMiliseconds = 1000.0;
  static const double kMicrosecondsToMiliseconds = 0.001;
  static const double kMilisecondsToSeconds = 0.001;
};
}//namespace arc_tools.

#endif