#ifndef PEG_KUKA_CMD_INTERFACE_H_
#define PEG_KUKA_CMD_INTERFACE_H_

#include <ros/ros.h>
#include "peg_hole_kuka/String_cmd.h"
#include "std_msgs/String.h"
#include <exploration_planner/String_cmd.h>

#include "kuka_action_client/String_cmd.h"
#include "particle_filter/String_cmd.h"
#include <map>

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
        UTILITY
    } cmd_type;


public:

    Cmd_interface(ros::NodeHandle&   nh,
                  const std::string& service_name,
                  const std::string& action_client_name,
                  const std::string& pf_client_name,
                  const std::string& exploration_client_name);

    bool service_callback(peg_hole_kuka::String_cmd::Request& req,peg_hole_kuka::String_cmd::Response &res);

private:

    void init_commands();

    bool call_action(const std::string& cmd);

    bool call_particle_filter(const std::string& cmd);

    bool call_utility(const std::string& cmd);

    bool call_planner(const std::string& cmd);

private:

    ros::ServiceServer                       service;

    ros::ServiceClient                       action_client;
    kuka_action_client::String_cmd           action_cmd;

    ros::ServiceClient                       particle_filter_client;
    particle_filter::String_cmd              particle_filter_cmd;

    ros::ServiceClient                       exploration_client;
    exploration_planner::String_cmd          exploration_cmd;

    std::map<std::string,cmd_type>           cmds;
    std::map<std::string,cmd_type>::iterator it;

};

#endif
