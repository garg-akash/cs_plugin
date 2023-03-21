#ifndef DATA_H
#define DATA_H

#include <string>
#include "ros/ros.h"
#include <interpolation.h>

using namespace std;
class Data {
  public:
    Data(ros::NodeHandle *nh, string s = "/move_group/display_planned_path");
    ~Data();

    void setMessage(string message);
    string getMessage() const;
    void getMoveitTraj(string message);
    void deployMoveitTraj();
    void setSamplingTime(double);

  private:
    string message ;
    ros::NodeHandle *nh_;
    Interpolation *interObj;
};

#endif // DATA_H