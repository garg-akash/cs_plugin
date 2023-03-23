#include <interpolation.h>

#include <cs_ros/realtime_control_client.h>

Interpolation::Interpolation(ros::NodeHandle *nh, double sampling_time, std::string hostip)
{
  this->nh_ = nh;
  sampling_time_ = sampling_time;
  total_time_ = 0;
  hostip_ = hostip;
  trajectory_acquired_ = false;
}

Interpolation::~Interpolation()
{
}

// double Interpolation::radToDeg(const double val_rad)
// {
//   double val_deg = val_rad * 180 / M_PI;
//   return val_deg; 
// }

// double Interpolation::degToRad(const double val_deg)
// {
//   double val_rad = val_deg * M_PI / 180;
//   return val_rad; 
// }

// std::vector<std::vector<double>> Interpolation::transpose(const std::vector<std::vector<double>> data) 
// {
//   std::vector<std::vector<double>> result(data[0].size(),
//                                         std::vector<double>(data.size()));
//   for (std::vector<double>::size_type i = 0; i < data[0].size(); i++) 
//       for (std::vector<double>::size_type j = 0; j < data.size(); j++) {
//           result[i][j] = data[j][i];
//       }
//   return result;
// }

void Interpolation::setSamplingTime(double tm)
{
  sampling_time_ = tm;
}

void Interpolation::setTrajectoryAcquiredFlag()
{
  trajectory_acquired_ = false;
}

double Interpolation::pos_inter(double t, const std::vector<double>& a)
{
  double tf[] = {pow(t,0), pow(t,1), pow(t,2), pow(t,3), pow(t,4), pow(t,5)};
  if(t == 0)
    return a[0];
  return (a[0] + a[1]*tf[1] + a[2]*tf[2] + a[3]*tf[3] + a[4]*tf[4] + a[5]*tf[5]);
}

double Interpolation::vel_inter(double t, const std::vector<double>& a)
{
  double tf[] = {pow(t,0), pow(t,1), pow(t,2), pow(t,3), pow(t,4), pow(t,5)};
  if(t == 0)
    return a[1];
  return (a[1] + 2*a[2]*tf[1] + 3*a[3]*tf[2] + 4*a[4]*tf[3] + 5*a[5]*tf[4]);
}

double Interpolation::acc_inter(double t, const std::vector<double>& a)
{
  double tf[] = {pow(t,0), pow(t,1), pow(t,2), pow(t,3), pow(t,4), pow(t,5)};
  if(t == 0)
    return 2*a[2];
  return (2*a[2] + 6*a[3]*tf[1] + 12*a[4]*tf[2] + 20*a[5]*tf[3]);
}

void Interpolation::getInterpolationFunctionsByJoint(std::vector<double>& a, double T, double q_start, double q_end, double dq_start, double dq_end, double ddq_start, double ddq_end)
{
  double q_delta = q_end - q_start;
  double tf[] = {pow(T,0), pow(T,1), pow(T,2), pow(T,3), pow(T,4), pow(T,5)};
  a.push_back(q_start);
  a.push_back(dq_start);
  a.push_back(ddq_start/2);
  a.push_back((20*q_delta - (8*dq_end + 12*dq_start) * tf[1] - (3*ddq_start - ddq_end) * tf[2]) / (2*tf[3]));
  a.push_back((-30*q_delta + (14*dq_end + 16*dq_start) * tf[1] + (3*ddq_start - 2*ddq_end) * tf[2]) / (2*tf[4]));
  a.push_back((12*q_delta - (dq_end + dq_start) * 6 * tf[1] - (ddq_start - ddq_end) * tf[2]) / (2*tf[5]));
}

double Interpolation::pos_inter_t(double t_request, std::map<double, func> pos_time_func)
{ 
  double t_previous = 0; 
  for(auto i : pos_time_func)
  { 
    if(t_request <= i.first)
    { 
      return pos_time_func[i.first](t_request - t_previous);
    } 
    t_previous = i.first; 
  }
  return 0; 
}

double Interpolation::vel_inter_t(double t_request, std::map<double, func> vel_time_func)
{
  double t_previous = 0;
  for(auto i : vel_time_func)
  {
    if(t_request <= i.first)
    {
      return vel_time_func[i.first](t_request - t_previous);
    }
    t_previous = i.first;
  }
  return 0; 
}

double Interpolation::acc_inter_t(double t_request, std::map<double, func> acc_time_func)
{
  double t_previous = 0;
  for(auto i : acc_time_func)
  {
    if(t_request <= i.first)
    {
      return acc_time_func[i.first](t_request - t_previous);
    }
    t_previous = i.first;
  }
  return 0; 
}

std::tuple<func_t, func_t> Interpolation::getInterpolationFunctions(const trajectory_msgs::JointTrajectory& jt, int ind, double& total_time)
{
  std::filebuf fb;
  if(ind == 0)
    fb.open ("joint0.txt",std::ios::out);
  else if(ind == 1)
    fb.open ("joint1.txt",std::ios::out);
  else if(ind == 2)
    fb.open ("joint2.txt",std::ios::out);
  else if(ind == 3)
    fb.open ("joint3.txt",std::ios::out);
  else if(ind == 4)
    fb.open ("joint4.txt",std::ios::out);
  else
    fb.open ("joint5.txt",std::ios::out);

  std::ostream fs(&fb);

  total_time = jt.points[jt.points.size()-1].time_from_start.toSec();
  std::map<double, func> pos_time_func, vel_time_func, acc_time_func;
  for(int j = 1; j < jt.points.size(); j++)
  {
    trajectory_msgs::JointTrajectoryPoint p_start = jt.points[j-1];
    trajectory_msgs::JointTrajectoryPoint p_end = jt.points[j];
    
    double time = p_end.time_from_start.toSec() - p_start.time_from_start.toSec();
    double q_start = p_start.positions[ind];
    double q_end = p_end.positions[ind];
    double dq_start = p_start.velocities[ind];
    double dq_end = p_end.velocities[ind];
    double ddq_start = p_start.accelerations[ind];
    double ddq_end = p_end.accelerations[ind];
    std::vector<double> a_coeff;

    getInterpolationFunctionsByJoint(a_coeff, time, q_start, q_end, dq_start, dq_end, ddq_start, ddq_end);
    pos_time_func.emplace(p_end.time_from_start.toSec(), std::bind(&Interpolation::pos_inter,this,std::placeholders::_1,a_coeff));
    vel_time_func.emplace(p_end.time_from_start.toSec(), std::bind(&Interpolation::vel_inter,this,std::placeholders::_1,a_coeff));
    acc_time_func.emplace(p_end.time_from_start.toSec(), std::bind(&Interpolation::acc_inter,this,std::placeholders::_1,a_coeff));

    fs << p_start.time_from_start.toSec() << "\t" << q_start << "\t" << dq_start << "\t" << ddq_start << "\n";
    if(j == jt.points.size() - 1)
      fs << p_end.time_from_start.toSec() << "\t" << q_end << "\t" << dq_end << "\t" << ddq_end << "\n";
  }
  fb.close();
  return std::make_tuple(std::bind(&Interpolation::pos_inter_t,this,std::placeholders::_1,pos_time_func), std::bind(&Interpolation::vel_inter_t,this,std::placeholders::_1,vel_time_func));
}


void Interpolation::subscriberAndInterpolate(std::string moveit_path_rostopic)
{
  std::cout << "Going to subscriber to topic\n";
  ros::Subscriber traj_sub_ = nh_->subscribe(moveit_path_rostopic, 100, &Interpolation::cbTrajInterpolation, this);
  std::cout << "After callback\n";
  ros::Rate r(10);
  while (!trajectory_acquired_)
  {
    ros::spinOnce();
    r.sleep();
  }
}

void Interpolation::cbTrajInterpolation(const moveit_msgs::DisplayTrajectory::ConstPtr& msg)
{
  std::cout << "In callback\n";

  sensor_msgs::JointState js_start = msg->trajectory_start.joint_state;
  int num_joints = js_start.position.size();

  for(int i = 0; i < msg->trajectory.size(); i++) // rn we have only one trajectory
  {
    func_t pos_t, vel_t, acc_t;
    for(int k = 0; k < num_joints; k++)
    {
      std::filebuf fb_;
      std::filebuf fb_vel_;
      if(k == 0)
      {
        fb_.open ("interjoint0.txt",std::ios::out);
        fb_vel_.open ("intervel0.txt",std::ios::out);
      }
      else if(k == 1)
      {
        fb_.open ("interjoint1.txt",std::ios::out);
        fb_vel_.open ("intervel1.txt",std::ios::out);
      }
      else if(k == 2)
      {
        fb_.open ("interjoint2.txt",std::ios::out);
        fb_vel_.open ("intervel2.txt",std::ios::out);
      }
      else if(k == 3)
      {
        fb_.open ("interjoint3.txt",std::ios::out);
        fb_vel_.open ("intervel3.txt",std::ios::out);
      }
      else if(k == 4)
      {
        fb_.open ("interjoint4.txt",std::ios::out);
        fb_vel_.open ("intervel4.txt",std::ios::out);
      }
      else
      {
        fb_.open ("interjoint5.txt",std::ios::out);
        fb_vel_.open ("intervel5.txt",std::ios::out);
      }

      std::ostream fs_(&fb_);
      std::ostream fs_vel_(&fb_vel_);

      std::cout << "___________Processing joint : " << k << "____________\n";
      std::vector<double> jpos;
      std::tie(pos_t, vel_t) = getInterpolationFunctions(msg->trajectory[i].joint_trajectory, k, total_time_);
      ROS_INFO("total time : [%f]", total_time_);
      int length = total_time_ / sampling_time_; // not always int??
      for(int l = 0; l < length; l++)
      {
        double tm = l * sampling_time_;
        jpos.push_back(pos_t(tm));
        fs_ << tm << "\t" << jpos.back() << "\n";
        // fs_vel_ << tm << "\t" << radToDeg(vel_t(tm)) << "\t" << radToDeg(vel_t(tm))/CS_JOINT_VELOCITY_LIMIT[k] << "\n";
      }
      jpos.push_back(pos_t(total_time_));
      fs_ << total_time_ << "\t" << jpos.back() << "\n";
      // fs_vel_ << total_time_ << "\t" << radToDeg(vel_t(total_time_)) << "\t" << radToDeg(vel_t(total_time_))/CS_JOINT_VELOCITY_LIMIT[k] << "\n";
      fb_.close();
      fb_vel_.close();
      joints_pos_.push_back(jpos); //fill joint by joint
    }
    std::cout << "___________All joints processed____________\n";      
    trajectory_acquired_ = true;
  }
}

void Interpolation::deployMoveitTraj()
{
  if (joints_pos_.size() == 0)
  {  
    std::cerr << "Please perform interpolation before calling deploy\n";
    return;
  }
  // joints_pos_ is joints X waypoints
  std::vector<std::vector<double>> joints_pos_trans = transpose(joints_pos_);

  RealTimeControlClient rc_client(nh_, hostip_, sampling_time_, total_time_, joints_pos_trans);
  rc_client.callServiceServer();
}

int main()
{
  return 0;
}
