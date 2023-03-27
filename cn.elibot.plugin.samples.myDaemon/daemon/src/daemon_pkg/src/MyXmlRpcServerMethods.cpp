#include <iostream>
#include "MyXmlRpcServerMethods.h"

using namespace std;

SetMessage::SetMessage(Data *_data) : data(_data)
{
  this->_signature = "s:s";
}

void SetMessage::execute(xmlrpc_c::paramList const &paramList, xmlrpc_c::value *const ret)
{
  string const msg(paramList.getString(0));
  paramList.verifyEnd(1);

  data->setMessage(msg);

  cout << "xmlRpc call set_message : " << msg << endl;

  *ret = xmlrpc_c::value_string(msg);
}

GetMessage::GetMessage(Data *_data) : data(_data)
{
  this->_signature = "s:";
}

void GetMessage::execute(xmlrpc_c::paramList const &paramList, xmlrpc_c::value *const ret)
{
  string const name(paramList.getString(0));
  paramList.verifyEnd(1);
  cout << "xmlRpc call get_message : " << data->getMessage() << endl;
  *ret = xmlrpc_c::value_string(data->getMessage());
}

GetMoveitTraj::GetMoveitTraj(Data *_data) : data(_data)
{
  this->_signature = "s:s";
}

void GetMoveitTraj::execute(xmlrpc_c::paramList const &paramList, xmlrpc_c::value *const ret)
{
  string const msg(paramList.getString(0));
  paramList.verifyEnd(1);

  data->getMoveitTraj(msg);

  cout << "xmlRpc call get_moveit_traj : " << msg << endl;

  *ret = xmlrpc_c::value_string(msg);
}

DeployMoveitTraj::DeployMoveitTraj(Data *_data) : data(_data)
{
  this->_signature = "s:s";
}

void DeployMoveitTraj::execute(xmlrpc_c::paramList const &paramList, xmlrpc_c::value *const ret)
{
  // string const msg(paramList.getString(0));
  // paramList.verifyEnd(1);

  data->deployMoveitTraj();

  cout << "xmlRpc call deploy_moveit_traj" << endl;

  *ret = xmlrpc_c::value_string(""); //return empty string
}

SetSamplingTime::SetSamplingTime(Data *_data) : data(_data)
{
  this->_signature = "s:s";
}

void SetSamplingTime::execute(xmlrpc_c::paramList const &paramList, xmlrpc_c::value *const ret)
{
  double msg =  paramList.getDouble(0);
  paramList.verifyEnd(1);

  data->setSamplingTime(msg);

  cout << "xmlRpc call set_sampling_time : " << msg << endl;

  *ret = xmlrpc_c::value_double(msg);
}

SetIP::SetIP(Data *_data) : data(_data)
{
  this->_signature = "s:s";
}

void SetIP::execute(xmlrpc_c::paramList const &paramList, xmlrpc_c::value *const ret)
{
  string const msg(paramList.getString(0));
  paramList.verifyEnd(1);

  data->setIP(msg);

  cout << "xmlRpc call set_ip : " << msg << endl;

  *ret = xmlrpc_c::value_string(msg);
}