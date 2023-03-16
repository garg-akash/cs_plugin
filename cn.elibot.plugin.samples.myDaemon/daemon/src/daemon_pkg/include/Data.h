#ifndef DATA_H
#define DATA_H

#include <string>
#include "ros/ros.h"
#include <interpolation.h>

using namespace std;
class Data {
  public:
    Data(ros::NodeHandle);
    ~Data();

    void setMessage(string message);
    string getMessage() const;
    void getMoveitTraj(string message);

  private:
    string message ;
    ros::NodeHandle nh_;
    Interpolation interObj;
};

#endif // DATA_H