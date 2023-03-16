#include "Data.h"
#include <sstream>


Data::Data(ros::NodeHandle nh) : message("hello world!"), nh_(nh) {
}

Data::~Data() {
}

string Data::getMessage() const{
    
    return this->message;
}

void Data::setMessage(string message) {
  this->message = message;
}

void Data::getMoveitTraj(string message) {
  this->message = "get moveit traj";
  interObj.subscriberAndInterpolate(nh_);
}

