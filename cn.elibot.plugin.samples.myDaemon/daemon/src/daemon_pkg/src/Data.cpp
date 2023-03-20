#include "Data.h"
#include <sstream>


Data::Data(ros::NodeHandle *nh, string s) {
  this->nh_ = nh;
  this->message = s;
  interObj = new Interpolation(this->nh_);
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
  // this->message = "get moveit traj";
  interObj->setTrajectoryAcquiredFlag();
  interObj->subscriberAndInterpolate(this->message);
}

void Data::setSamplingTime(double tm) {
  interObj->setSamplingTime(tm);;
}
