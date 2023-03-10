#ifndef MyXMLRPCSEVERMETHODS_H
#define MyXMLRPCSEVERMETHODS_H

#include "xmlrpc-c/base.hpp"
#include "xmlrpc-c/registry.hpp"
#include "xmlrpc-c/server_abyss.hpp"
#include "Data.h"

class SetMessage: public xmlrpc_c::method {
  public:
    SetMessage(Data* data);
    void execute(xmlrpc_c::paramList const& paramList, xmlrpc_c::value * const retvalP);
  private:
    SetMessage(); 
    Data* data;
};

class GetMessage: public xmlrpc_c::method {
  public:
    GetMessage(Data* data);
    void execute(xmlrpc_c::paramList const& paramList, xmlrpc_c::value * const retvalP);
  private:
    GetMessage();
    Data* data;
};

#endif // MyXMLRPCSEVERMETHODS_H