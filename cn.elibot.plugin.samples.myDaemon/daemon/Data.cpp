#include "Data.h"
#include <sstream>


Data::Data() : message("hello world!") {
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
}

