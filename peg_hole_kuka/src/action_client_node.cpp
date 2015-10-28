#include <ros/ros.h>

#include "kuka_action_client/action_client_cmd_interface.h"
#include "kuka_action_client/kuka_action_client.h"
#include "ros_param_parser/ros_param_parser.h"
#include "peg_hole_kuka/action_client.h"

int main(int argc, char** argv)
{

    ros::init(argc, argv,"control_cmd_interface");
    ros::NodeHandle nh("control_cmd_interface");

    std::string node_name = ros::this_node::getName();

    std::map<std::string,std::string> param_name_value;
    param_name_value[node_name + "/speech_topic"]          = "";
    param_name_value[node_name + "/action_service_name"]   = "";
    param_name_value[node_name + "/cmd_service_name"]      = "";
    param_name_value[node_name + "/action_server_name"]    = "";

    if(!pps::parser_string(nh,param_name_value)){
        ROS_ERROR("failed to parse all parameters!");
        return -1;
    }
    std::string speech_topic          =  param_name_value[node_name + "/speech_topic"];
    std::string action_serivce_name   = param_name_value[node_name + "/action_service_name"];
    std::string cmd_service_name      = param_name_value[node_name + "/cmd_service_name"];
    std::string action_server_name    =  param_name_value[node_name + "/action_server_name"];

    /** ------------- Initialise Action Client & Set Action-Goals ------------- **/

    ac::Kuka_action_client  kuka_action_client(action_server_name);

    PEG_action_client peg_action_clients;
    peg_action_clients.initialise();

    kuka_action_client.push_back(peg_action_clients.goals);

    /**  ------------- Initialise Control cmd  interface  ------------- **/

    ac::Action_client_cmd_interface action_client_cmd_interface(nh,kuka_action_client,action_serivce_name,cmd_service_name);
    action_client_cmd_interface.init_nl_subscriber(speech_topic);


    ros::spin();


    return 0;
}
