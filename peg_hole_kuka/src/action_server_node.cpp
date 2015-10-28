#include "kuka_action_server/action_server.h"
#include "ros_param_parser/ros_param_parser.h"

#include "kuka_common_action_server/kuka_goto_cart_as.h"
#include "kuka_common_action_server/kuka_grav_as.h"
#include "exploration_planner/simple_exploration.h"

int main(int argc, char** argv)
{

    // ----------- Launch ros node ----------------------

    ros::init(argc, argv, "plan2ctrl");
    ROS_INFO("Initializing Server");
    ros::NodeHandle nh;

    // ----------- Get parameters (parameter sever) ------

    std::string node_name = ros::this_node::getName();
    std::map<std::string,std::string> param_name_value;
    param_name_value[node_name + "/action_server_name"]  = "";
    param_name_value[node_name + "/world_frame"]         = "";


    if(!pps::parser_string(nh,param_name_value)){
        ROS_ERROR("failed to parse all parameters!");
        return -1;
    }
    pps::parser_print(param_name_value);
    std::string action_server_name  =  param_name_value[node_name + "/action_server_name"];
    std::string world_frame         =   param_name_value[node_name + "/world_frame"];

    /**  ------------- Initialise control policies ------------- **/

    asrv::Action_ee_initialiser ee_init;
    ee_init.action_name     = "goto_cart";
    asrv::Kuka_goto_cart_as kuka_goto_cart_as(nh,ee_init);


    // Belief space planner
    asrv::Action_ee_initialiser sbp_ee_init;
    sbp_ee_init.action_name             =   "simple_bel_planner";
    std::string bel_feature_topic_name  =   "/mode_feature";
    std::string frame_id                =   world_frame;

    belief::Simple_planner simple_planner(nh,bel_feature_topic_name,frame_id,sbp_ee_init);


    /**  ------------- Initialise Action Server ------------- **/

    asrv::Action_server action_server(nh,action_server_name);


    /**  ------------- Push back policies ------------- **/

    action_server.push_back(&kuka_goto_cart_as,"goto_cart");
    action_server.push_back(&simple_planner,"simple_bel_planner");

    ros::spin();


    return 0;
}
