#ifndef DATA_H
#define DATA_H

#include <string>

using namespace std;
class Data {
  public:
    Data();
    ~Data();

    void setMessage(string message);
    string getMessage() const;
    void getMoveitTraj(string message);

  private:
    string message ;
};

#endif // DATA_H