#include <ros/ros.h>

#include "peg_hole_kuka/cmd_interface.h"
#include "kuka_action_client/action_client_cmd_interface.h"
#include "peg_hole_kuka/console.h"
#include <memory>
#include <functional>

int main(int argc,char** argv)
{

    ros::init(argc,argv,"cmd_interface");
    ros::NodeHandle nh;


    /**  ------------- Overlord command interface  ------------- **/

    std::string service_name            = "cmd_overlord";
    std::string action_client_name      = "/kuka_action_client/kuka_action_cmd";    /// to communicate with action client (call actions)
    std::string pf_client_name          = "/pf_service";
    std::string exploration_client_name = "/bel_simple_planner_cmd";
    std::string peg_policy_client_name  = "/cmd_peg_policy";                        /// to communicate with peg_hole_policy (call sub-actions)
    std::string voice_topic_name        = "/allegroHand/lib_cmd";                   /// listen for voice
    std::string record_topic_name       =  "/record/cmd";

    Cmd_interface cmd_interface(nh,
                                service_name,
                                action_client_name,
                                pf_client_name,
                                exploration_client_name,
                                peg_policy_client_name,
                                voice_topic_name,
                                record_topic_name);




    cmdi::PegConsole peg_console(nh,cmd_interface);

    peg_console.get_console();


    peg_console.AddConsoleCommand("plug_search");
    peg_console.AddConsoleCommand("gmm");


    peg_console.AddConsoleCommand("grav_comp");
    peg_console.AddConsoleCommand("go_front");
    peg_console.AddConsoleCommand("go_left");
    peg_console.AddConsoleCommand("go_peg_right");
    peg_console.AddConsoleCommand("home");
    peg_console.AddConsoleCommand("disconnect");
    peg_console.AddConsoleCommand("insert");

    peg_console.AddConsoleCommand("go_table");
    peg_console.AddConsoleCommand("go_socket");

    peg_console.AddConsoleCommand("stop");
    peg_console.AddConsoleCommand("bias");

    peg_console.AddConsoleCommand("pf_reset");
    peg_console.AddConsoleCommand("pf_start");

    peg_console.AddConsoleCommand("record");

    peg_console.AddConsoleCommand("open_loop");
    peg_console.AddConsoleCommand("passive_ds");

    cmd_interface.set_console(peg_console.get_console());


    peg_console.start();



    ros::Rate rate(50);
    while(ros::ok()){

        peg_console.ConsoleUpdate();

        rate.sleep();
        ros::spinOnce();
    }

    peg_console.stop();


    return 0;
}
