#include "peg_hole_kuka/cmd_interface.h"

Cmd_interface::Cmd_interface(ros::NodeHandle &nh,
                             const std::string& service_name,
                             const std::string& action_client_name,
                             const std::string& pf_client_name,
                             const std::string& exploration_client_name)
{
    service                 = nh.advertiseService(service_name,&Cmd_interface::service_callback,this);


    action_client           = nh.serviceClient<kuka_action_client::String_cmd>(action_client_name);
    particle_filter_client  = nh.serviceClient<particle_filter::String_cmd>(pf_client_name);
    exploration_client      = nh.serviceClient<exploration_planner::String_cmd>(exploration_client_name);


    init_commands();

}

void Cmd_interface::init_commands(){

    /*cmds["home"]                = cmd_type::ACTION;
    cmds["grav_on"]             = cmd_type::ACTION;
    cmds["grav_off"]            = cmd_type::ACTION;
    cmds["start"]               = cmd_type::ACTION;
    cmds["stop"]                = cmd_type::ACTION;*/

    cmds["plug"]                = cmd_type::ACTION;

    cmds["simple_bel_planner"]  = cmd_type::ACTION;
    cmds["planner pause"]       = cmd_type::PLANNER;


    cmds["pf reset"]            = cmd_type::PF;
    cmds["pf start"]            = cmd_type::PF;
    cmds["print"]               = cmd_type::UTILITY;
}

bool Cmd_interface::service_callback(peg_hole_kuka::String_cmd::Request& req,peg_hole_kuka::String_cmd::Response &res){

    std::string cmd = req.cmd;
    it = cmds.find(cmd);

    if(it != cmds.end()){
        cmd_type cmd_t = it->second;

        switch(cmd_t){
        case cmd_type::ACTION:
        {
            return call_action(cmd);
            break;
        }
        case cmd_type::PF:
        {
            return call_particle_filter(cmd);
            break;
        }
        case cmd_type::FT:
        {
            break;
        }
        case cmd_type::PLANNER:
            return call_planner(cmd);
            break;
        case cmd_type::UTILITY:
        {
            return call_utility(cmd);
            break;
        }
        default:
        {
            ROS_ERROR("no such cmd_type [%d] defined",static_cast<int>(cmd_t));
            return false;
            break;
        }
        }

    }else{

        res.res = "Cmd_interface::service_callback      Error no such cmd [" + cmd + "] defined! ";
        return false;
    }

}

bool Cmd_interface::call_action(const std::string& cmd){
    action_cmd.request.cmd = cmd;
    if (!action_client.call(action_cmd)){
        ROS_ERROR("Cmd_interface::service_callback            Failed to call service action [%s]",cmd.c_str());
        return false;
    }else{
        return true;
    }
}

bool Cmd_interface::call_planner(const std::string& cmd){

    if(cmd == "planner pause"){
        exploration_cmd.request.cmd = "pause";
        exploration_client.call(exploration_cmd);
        return true;
    }else{
        ROS_ERROR("no such cmd [%s] declared in Cmd_interface",cmd.c_str());
        return false;
    }
}

bool Cmd_interface::call_particle_filter(const std::string& cmd){
      if(cmd == "pf reset"){
            particle_filter_cmd.request.cmd = "reset";
      }else if(cmd == "pf start"){
            particle_filter_cmd.request.cmd = "start";
      }else{
          ROS_ERROR("no such cmd [%s] declared in Cmd_interface",cmd.c_str());
          return false;
      }



    if (!particle_filter_client.call(particle_filter_cmd)){
        ROS_ERROR("Cmd_interface::service_callback            Failed to call service particle_filter [%s]",cmd.c_str());
        return false;
    }else{
        return true;
    }
}

bool Cmd_interface::call_utility(const std::string& cmd){
    if(cmd == "print"){
        std::cout<< std::endl;
        std::cout<< " === cmd interface === " << std::endl;
        for(it = cmds.begin(); it != cmds.end();it++){
        std::cout<< "  " << it->first << std::endl;

        }
        std::cout<<std::endl;
        return true;
    }else{
        return false;
    }

}

