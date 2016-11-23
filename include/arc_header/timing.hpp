#ifndef TIMING_ARC_HEADER_HPP
#define TIMING_ARC_HEADER_HPP

class Clock {

 public:
  Clock() { start(); firstStep_ = true;}
  double getTimestep(){
  	if(firstStep_){
  		lastTime_ = getTime();
      firstStep_ = false;
  	}
  	currentTime_ = getTime();
  	timeStep_ = currentTime_ - lastTime_;
  	lastTime_ = getTime();
  	return timeStep_*0.001;
  }
  void start(){ gettimeofday(&realTimeStart_, NULL); }
 private:
  double getTime() { takeTime(); return getRealTime(); }
  double getRealTime() { return realTime_ms_;}
  void takeTime(){
  	//Updating cpu Time
    struct timeval end;
    gettimeofday(&end, NULL);
    long seconds, useconds;
    seconds  = end.tv_sec  - realTimeStart_.tv_sec;
    useconds = end.tv_usec - realTimeStart_.tv_usec;
    realTime_ms_ = (seconds * kSecondsToMiliseconds +
        useconds * kMicrosecondsToMiliseconds) + 0.5;
  }
  struct timeval realTimeStart_;
  double realTime_ms_;
  bool firstStep_;
  double lastTime_, currentTime_, timeStep_;
  static const double kSecondsToMiliseconds = 1000.0;
  static const double kMicrosecondsToMiliseconds = 0.001;
};

#endif