#include "kuka_action_server/action_server.h"
#include "kuka_action_client/ros_param_parser.h"

#include "kuka_common_action_server/kuka_goto_cart_as.h"
#include "kuka_common_action_server/kuka_grav_as.h"
#include "exploration_planner/belief_gmm_planner.h"
#include "exploration_planner/simple_exploration.h"
#include "robot_planners/gmmPlanner.h"
#include "robot_planners/planner_ee.h"

#include "peg_hole_policy/peg_hole_policy.h"
#include "simple_actions/linear_cart_action.h"


int main(int argc, char** argv)
{

    // ----------- Launch ros node ----------------------

    ros::init(argc, argv,"");
    ROS_INFO("Initializing Server");
    ros::NodeHandle nh;

    // ----------- Get parameters (parameter sever) ------

    std::string node_name = ros::this_node::getName();
    std::map<std::string,std::string> param_name_value;
    param_name_value[node_name + "/action_server_name"]  = "";
    param_name_value[node_name + "/world_frame"]         = "";
    param_name_value[node_name + "/path_sensor_model"]   = "";

    param_name_value[node_name + "/ft_topic"]            = "";
    param_name_value[node_name + "/classifier_topic"]    = "";
    param_name_value[node_name + "/F_topic"]             = "";
    param_name_value[node_name + "/gmm_param_path"]      = "";


    if(!pps::parser_string(nh,param_name_value)){
        ROS_ERROR("failed to parse all parameters!");
        return -1;
    }
    pps::parser_print(param_name_value);
    std::string action_server_name  =  param_name_value[node_name + "/action_server_name"];
    std::string world_frame         =  param_name_value[node_name + "/world_frame"];
    std::string path_sensor_model   =  param_name_value[node_name + "/path_sensor_model"];
    std::string ft_topic            =  param_name_value[node_name + "/ft_topic"];
    std::string classifier_topic    =  param_name_value[node_name + "/classifier_topic"];
    std::string F_topic             =  param_name_value[node_name + "/F_topic"];
    std::string path_gmm_param      =  param_name_value[node_name + "/gmm_param_path"];


    /**  ------------- Initialise control policies ------------- **/


    belief::Gmm_planner_initialiser init;
    init.belief_state_size      = 4;
    init.bel_feature_topic      = F_topic;
    init.ft_classifier_topic    = classifier_topic;
    init.sensor_topic           = ft_topic;
    init.world_frame            = world_frame;
    init.path_parameters        = path_gmm_param;
    belief::Gmm_planner gmm_planner(nh,init);

    Peg_world_wrapper   peg_world_wrapper(nh,"peg_hole_kuka_action",path_sensor_model,world_frame,"lwr_peg_link");

   // Peg_sensor_model&   peg_sensor_model = *(peg_world_wrapper.peg_sensor_model.get());


    ph_policy::Peg_hole_policy peg_hole_policy(nh,path_sensor_model,world_frame,gmm_planner,peg_world_wrapper);

   // simple_actions::Linear_cart_action linear_cart_action(nh);



    /**  ------------- Initialise Action Server ------------- **/

    asrv::Action_server action_server(nh,action_server_name);


    /**  ------------- Push back policies ------------- **/

    action_server.push_back(&peg_hole_policy,"plug_search");
    //action_server.push_back(&linear_cart_action,"linear");


    ROS_INFO("ACTION SERVER STARTED!");
    ros::spin();


    return 0;
}
