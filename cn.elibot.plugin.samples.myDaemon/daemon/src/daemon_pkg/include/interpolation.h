#ifndef INTERPOLATION_H
#define INTERPOLATION_H

#include "ros/ros.h"
#include "moveit_msgs/DisplayTrajectory.h"

class Interpolation
{
private:
  /* data */
public:
  Interpolation(/* args */);
  ~Interpolation();
  void subscriberAndInterpolate(ros::NodeHandle);
  void cbTrajInterpolation(const moveit_msgs::DisplayTrajectory::ConstPtr& msg);
};

#endif // INTERPOLATION_H