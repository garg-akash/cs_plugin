#ifndef INTERPOLATION_H
#define INTERPOLATION_H

#include "ros/ros.h"
#include "moveit_msgs/DisplayTrajectory.h"
#include <fstream>
#include <sstream>

typedef std::function<double(double)> func; // binding the second argument using placeholder
typedef std::function<double(double)> func_t; // binding the second argument using placeholder

// already defined in realtime_control_client.h
// const int CS_JOINT_VELOCITY_LIMIT[] = {30, 150, 180, 230, 230, 30}; // joint velocity limit for CS 

class Interpolation
{
private:
  ros::NodeHandle *nh_;
  std::string hostip_;
  double sampling_time_;
  double total_time_;
  bool trajectory_acquired_;
  std::vector<std::vector<double>> joints_pos_;
public:
  Interpolation(ros::NodeHandle *nh, double sampling_time = 0.004, std::string hostip = "192.168.133.133");
  ~Interpolation();
  void subscriberAndInterpolate(std::string);
  void cbTrajInterpolation(const moveit_msgs::DisplayTrajectory::ConstPtr& msg);
  std::tuple<func_t, func_t> getInterpolationFunctions(const trajectory_msgs::JointTrajectory& jt, int ind, double& total_time);
  double acc_inter_t(double t_request, std::map<double, func> acc_time_func);
  double vel_inter_t(double t_request, std::map<double, func> vel_time_func);
  double pos_inter_t(double t_request, std::map<double, func> pos_time_func);
  void getInterpolationFunctionsByJoint(std::vector<double>& a, double T, double q_start, double q_end, double dq_start, 
                                                      double dq_end, double ddq_start, double ddq_end);
  double acc_inter(double t, const std::vector<double>& a);
  double vel_inter(double t, const std::vector<double>& a);
  double pos_inter(double t, const std::vector<double>& a);
  // double degToRad(const double val_deg);
  // double radToDeg(const double val_rad);
  // std::vector<std::vector<double>> transpose(const std::vector<std::vector<double>>);
  void setIP(std::string);
  void setSamplingTime(double);
  void setTrajectoryAcquiredFlag();
  void deployMoveitTraj();
};

#endif // INTERPOLATION_H