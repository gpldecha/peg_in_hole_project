#include <ros/ros.h>

#include "peg_hole_kuka/cmd_interface.h"
#include "kuka_action_client/action_client_cmd_interface.h"

int main(int argc,char** argv)
{

    ros::init(argc,argv,"cmd_interface");
    ros::NodeHandle nh;


    /**  ------------- Overlord command interface  ------------- **/

    std::string service_name            = "cmd_overlord";
    std::string action_client_name      = "/control_cmd_interface/kuka_action_cmd";
    std::string pf_client_name          = "/pf_service";
    std::string exploration_client_name = "/bel_simple_planner_cmd";

    Cmd_interface cmd_interface(nh,
                                service_name,
                                action_client_name,
                                pf_client_name,
                                exploration_client_name);



    ros::spin();


    return 0;
}
