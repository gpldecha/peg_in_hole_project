#ifndef PEG_KUKA_CMD_INTERFACE_H_
#define PEG_KUKA_CMD_INTERFACE_H_

#include <ros/ros.h>
#include "peg_hole_kuka/String_cmd.h"
#include "std_msgs/String.h"
#include <exploration_planner/String_cmd.h>
#include <peg_hole_policy/String_cmd.h>

#include "kuka_action_client/String_cmd.h"
#include "particle_filter/String_cmd.h"
#include "netft_rdt_driver/netft_rdt_bias.h"
#include "netft_rdt_driver/String_cmd.h"
#include "record_ros/String_cmd.h"
#include <map>
#include "console/Console.h"

/**
 *  Command Interface for peg in hole task
 *
 *   All terminal command are handled here.
 *    - Action client listener
 *    - Particle filter listener
 *    - Record
 *
 */


class Cmd_interface{

public:

    typedef enum cmd_type{
        ACTION,
        PF,
        FT,
        PLANNER,
        UTILITY,
        PEG_POLICY,
        RECORD
    } cmd_type;

    class cmd_info{
        public:
        cmd_info(){}
        cmd_info(const::std::string& cmd_name,const cmd_type c_type):
        cmd_name(cmd_name),c_type(c_type){}
        std::string cmd_name;
        cmd_type  c_type;
    };

public:

    Cmd_interface(ros::NodeHandle&   nh,
                  const std::string& service_name,
                  const std::string& action_client_name,
                  const std::string& pf_client_name,
                  const std::string& exploration_client_name,
                  const std::string& peg_policy_client_name,
                  const std::string& voice_topic_name,
                  const string &record_topic_name);

    bool service_callback(peg_hole_kuka::String_cmd::Request& req,peg_hole_kuka::String_cmd::Response &res);

    void set_console(Console* console);

private:

    void nl_command_callback(const std_msgs::String::ConstPtr& msg);

    void init_commands();

    bool call_netft(const std::string& cmd, std::string& res);

    bool call_peg_policy(const std::string& cmd, std::string &res);

    bool call_action(const std::string& cmd);

    bool call_particle_filter(const std::string& cmd);

    bool call_utility(const std::string& cmd, std::string& res);

    bool call_planner(const std::string& cmd);

    bool call_record(const std::string& cmd);


private:

    ros::ServiceServer                       service;

    ros::ServiceClient                       action_client;
    kuka_action_client::String_cmd           action_cmd;

    ros::ServiceClient                       particle_filter_client;
    particle_filter::String_cmd              particle_filter_cmd;

    ros::ServiceClient                       exploration_client;
    exploration_planner::String_cmd          exploration_cmd;

    ros::ServiceClient                       netft_client;
    netft_rdt_driver::String_cmd             netft_cmd;


    ros::ServiceClient                       peg_policy_client;
    peg_hole_policy::String_cmd              peg_hole_policy;

    ros::ServiceClient                       record_client;
    record_ros::String_cmd                   record_cmd;


    ros::Subscriber                          nl_subscriber; /// natural language voice interface


    Console*                                 console;

    std::map<std::string,cmd_info>           cmds;
    std::map<std::string,cmd_info>::iterator it;

    peg_hole_kuka::String_cmd::Request       request;
    peg_hole_kuka::String_cmd::Response      response;

};

#endif
