#include "socket_table_broadcaster/services.h"


namespace sock_tab{

Services::Services(ros::NodeHandle &node, Save &save):
save(save){
   service_cmd       = node.advertiseService("string_cmd",&Services::service_str_callback,this);
}

bool Services::service_str_callback(socket_table_broadcaster::String_cmd::Request &req, socket_table_broadcaster::String_cmd::Response &resp){
    if(req.str == "save"){
        save.save();
        return true;
    }else{
        return false;
    }
}





}
