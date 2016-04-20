#include <ros/ros.h>

#include "lwr_ros_client/action_client_cmd_interface.h"
#include "lwr_ros_client/ros_param_parser.h"
#include "peg_hole_kuka/client.h"

int main(int argc, char** argv)
{

    ros::init(argc, argv,"kuka_action_client");
    ros::NodeHandle nh("kuka_action_client");

    std::string node_name = ros::this_node::getName();

    std::map<std::string,std::string> param_name_value;
    param_name_value[node_name + "/speech_topic"]          = "";
    param_name_value[node_name + "/action_service_name"]   = "";
    param_name_value[node_name + "/cmd_service_name"]      = "";
    param_name_value[node_name + "/action_server_name"]    = "";

    if(!pps::Parser::parser_string(nh,param_name_value)){
        ROS_ERROR("failed to parse all parameters!");
        return -1;
    }
    std::string speech_topic          =  param_name_value[node_name + "/speech_topic"];
    std::string action_serivce_name   =  param_name_value[node_name + "/action_service_name"];
    std::string cmd_service_name      =  param_name_value[node_name + "/cmd_service_name"];
    std::string action_server_name    =  param_name_value[node_name + "/action_server_name"];

    /** ------------- Initialise Action Client & Set Action-Goals ------------- **/

  //   ac::Kuka_action_client  kuka_action_client(action_server_name);


 //    kuka_action_client.push_back(peg_action_clients.goals);


    /**  ------------- Initialise Service, Voice & Cmd interface  -------------
     *  The control command interface is an interface to the action client.
     *  It provied a ros service and a voice command interface such to
     *  command the client server to send desired action requests to the action server.
     */
    // ac::Action_client_cmd_interface action_cmd_interface(nh,kuka_action_client,action_serivce_name,cmd_service_name);
    // action_cmd_interface.init_nl_subscriber(speech_topic);

     ROS_INFO("action CLIENT started!");
     ros::spin();



    return 0;
}
