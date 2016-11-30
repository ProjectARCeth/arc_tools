#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(100);

  tf::TransformBroadcaster broadcaster;
  
  //double start = ros::Time::now().toSec();

  while(n.ok()){
    double time_difference = ((ros::Time::now().toSec)-start);
    ROS_INFO("The time difference is: %f",time_difference);
    broadcaster.sendTransform(
      tf::StampedTransform(
              tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(10.0*time_difference, 0.0, 0.0)),
           ///tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(10.0, 0.0, 0.0)),
        ros::Time::now(),"world", "velodyne"));
    r.sleep();
  }
}
