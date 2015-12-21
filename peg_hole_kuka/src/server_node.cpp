#include "kuka_action_server/action_server.h"
#include "kuka_action_client/ros_param_parser.h"

#include "kuka_common_action_server/kuka_goto_cart_as.h"
#include "kuka_common_action_server/kuka_grav_as.h"
#include "exploration_planner/belief_gmm_planner.h"
#include "exploration_planner/simple_exploration.h"
#include "robot_planners/gmmPlanner.h"
#include "robot_planners/planner_ee.h"

#include "peg_hole_policy/peg_hole_policy.h"


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
    param_name_value[node_name + "/path_sensor_model"]   = "";


    if(!pps::parser_string(nh,param_name_value)){
        ROS_ERROR("failed to parse all parameters!");
        return -1;
    }
    pps::parser_print(param_name_value);
    std::string action_server_name  =  param_name_value[node_name + "/action_server_name"];
    std::string world_frame         =  param_name_value[node_name + "/world_frame"];
    std::string path_sensor_model   =  param_name_value[node_name + "/path_sensor_model"];

    /**  ------------- Initialise control policies ------------- **/


    ph_policy::Peg_hole_policy peg_hole_policy(nh,path_sensor_model,world_frame);



    /**  ------------- Initialise Action Server ------------- **/

    asrv::Action_server action_server(nh,action_server_name);


    /**  ------------- Push back policies ------------- **/

    action_server.push_back(&peg_hole_policy,"plug_search");

    ROS_INFO("ACTION SERVER STARTED!");
    ros::spin();


    return 0;
}
