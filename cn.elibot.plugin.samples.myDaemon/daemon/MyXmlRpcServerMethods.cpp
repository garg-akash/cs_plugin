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