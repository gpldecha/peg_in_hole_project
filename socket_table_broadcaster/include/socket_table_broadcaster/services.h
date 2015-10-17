#ifndef SERVICES_H_
#define SERVICES_H_

#include <ros/ros.h>

#include <socket_table_broadcaster/String_cmdRequest.h>
#include <socket_table_broadcaster/String_cmdResponse.h>
#include <socket_table_broadcaster/String_cmd.h>

#include "socket_table_broadcaster/save.h"

#include <tf/LinearMath/Transform.h>

namespace sock_tab{

class Services{

public:

    Services(ros::NodeHandle& node,sock_tab::Save& save);

private:

   bool service_str_callback(socket_table_broadcaster::String_cmd::Request &req, socket_table_broadcaster::String_cmd::Response& resp);

private:

    ros::ServiceServer            service_cmd;
    Save                          save;




};

}

#endif
