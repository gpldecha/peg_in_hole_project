#include "lwr_ros_client/action_client_cmd_interface.h"
#include "lwr_ros_client/kuka_action_client.h"
#include "lwr_ros_client/ros_param_parser.h"
#include "lwr_ros_action/joint_action.h"


//#include "exploration_planner/belief_gmm_planner.h"
//#include "exploration_planner/simple_exploration.h"
//#include "robot_planners/gmmPlanner.h"
//#include "robot_planners/planner_ee.h"

#include "peg_hole_policy/peg_hole_policy.h"
//#include "simple_actions/linear_cart_action.h"


int main(int argc, char** argv)
{

    // ----------- Launch ros node ----------------------

    ros::init(argc, argv,"");
    ROS_INFO("Initializing Server");
    ros::NodeHandle nh;

    // ----------- Get parameters (parameter sever) ------

    std::string node_name = ros::this_node::getName();
    std::map<std::string,std::string> param_name_value;
    param_name_value[node_name + "/world_frame"]         = "";
    param_name_value[node_name + "/path_sensor_model"]   = "";

    param_name_value[node_name + "/ft_topic"]            = "";
    param_name_value[node_name + "/classifier_topic"]    = "";
    param_name_value[node_name + "/F_topic"]             = "";
    param_name_value[node_name + "/gmm_param_path"]      = "";

    param_name_value[node_name + "/speech_topic"]        = "";
    param_name_value[node_name + "/action_service"]      = "";



    if(!pps::Parser::parser_string(nh,param_name_value)){
        ROS_ERROR("failed to parse all parameters!");
        return -1;
    }
    pps::Parser::parser_print(param_name_value);
    std::string world_frame         =  param_name_value[node_name + "/world_frame"];
    std::string path_sensor_model   =  param_name_value[node_name + "/path_sensor_model"];
    std::string ft_topic            =  param_name_value[node_name + "/ft_topic"];
    std::string classifier_topic    =  param_name_value[node_name + "/classifier_topic"];
    std::string F_topic             =  param_name_value[node_name + "/F_topic"];
    std::string path_gmm_param      =  param_name_value[node_name + "/gmm_param_path"];

    std::string speech_topic        =  param_name_value[node_name + "/speech_topic"];
    std::string action_serivce      =  param_name_value[node_name + "/action_service"];

    ac::Kuka_action_client kuka_action_client;
    std::map<std::string,ac::Base_action*> actions;


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

    ph_policy::Peg_hole_policy peg_hole_policy(nh,world_frame,gmm_planner,peg_world_wrapper);

    actions["plug_search"] = &peg_hole_policy;
    kuka_action_client.push_back(actions);


   /**  ------------- Initialise Service, Voice & Cmd interface  -------------
    *  The control command interface is an interface to the action client.
    *  It provied a ros service and a voice command interface such to
    *  command the client server to send desired action requests to the action server.
    */
     ac::Action_client_cmd_interface action_cmd_interface(nh,kuka_action_client,action_serivce);
     action_cmd_interface.init_nl_subscriber(speech_topic);

     ROS_INFO("action CLIENT started!");
     ros::spin();


    return 0;
}
