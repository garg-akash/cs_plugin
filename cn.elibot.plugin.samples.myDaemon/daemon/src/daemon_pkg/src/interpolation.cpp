#include <interpolation.h>

Interpolation::Interpolation(/* args */)
{
}

Interpolation::~Interpolation()
{
}

void Interpolation::subscriberAndInterpolate(ros::NodeHandle nh)
{
  std::cout << "Going to subscriber to topic\n";
  ros::Subscriber traj_sub_ = nh.subscribe("/move_group/display_planned_path", 100, &Interpolation::cbTrajInterpolation, this);
}

void Interpolation::cbTrajInterpolation(const moveit_msgs::DisplayTrajectory::ConstPtr& msg)
{
  ROS_INFO("frequency : ");
}
