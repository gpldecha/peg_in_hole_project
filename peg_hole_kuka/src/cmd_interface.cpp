#include "peg_hole_kuka/cmd_interface.h"

Cmd_interface::Cmd_interface(ros::NodeHandle &nh,
                             const std::string& service_name,
                             const std::string& action_client_name,
                             const std::string& pf_client_name,
                             const std::string& exploration_client_name,
                             const std::string& peg_policy_client_name,
                             const std::string& voice_topic_name)
{
    service                 = nh.advertiseService(service_name,&Cmd_interface::service_callback,this);


    action_client           = nh.serviceClient<kuka_action_client::String_cmd>(action_client_name);
    particle_filter_client  = nh.serviceClient<particle_filter::String_cmd>(pf_client_name);
    exploration_client      = nh.serviceClient<exploration_planner::String_cmd>(exploration_client_name);
    peg_policy_client       = nh.serviceClient<peg_hole_policy::String_cmd>(peg_policy_client_name);

    nl_subscriber           = nh.subscribe(voice_topic_name,1,&Cmd_interface::nl_command_callback,this);

    init_commands();

}

void Cmd_interface::init_commands(){

    cmds["joint_imp"]           = cmd_info("joint_imp",cmd_type::ACTION);
    cmds["safe_grav_comp"]      = cmd_info("safe_grav_comp",cmd_type::ACTION);
    cmds["disconnect"]          = cmd_info("disconnect",cmd_type::ACTION);

    cmds["disconnect_1"]          = cmd_info("disconnect_1",cmd_type::ACTION);
    cmds["disconnect_2"]          = cmd_info("disconnect_2",cmd_type::ACTION);




    cmds["go_connect_pos"]      = cmd_info("go_connect_pos",cmd_type::ACTION);
    cmds["go_connect_vel"]      = cmd_info("go_connect_vel",cmd_type::ACTION);

    cmds["go_left"]             = cmd_info("go_left",cmd_type::ACTION);
    cmds["go_front"]            = cmd_info("go_front",cmd_type::ACTION);

    cmds["stop"]                = cmd_info("stop",cmd_type::ACTION);

    cmds["home"]                = cmd_info("home",cmd_type::ACTION);



    cmds["joint_imp_s"]         = cmd_info("joint_imp_s",cmd_type::ACTION);
    cmds["plug_search"]         = cmd_info("plug_search",cmd_type::ACTION);

    cmds["go_table"]            = cmd_info("go_table",cmd_type::PEG_POLICY);
    cmds["go_socket"]           = cmd_info("go_socket",cmd_type::PEG_POLICY);
    cmds["insert"]              = cmd_info("insert",cmd_type::PEG_POLICY);
    cmds["pause"]               = cmd_info("pause",cmd_type::PEG_POLICY);

    cmds["pf reset"]            = cmd_info("pf reset",cmd_type::PF);
    cmds["pf start"]            = cmd_info("pf start",cmd_type::PF);
    cmds["print"]               = cmd_info("print",cmd_type::UTILITY);
}


void Cmd_interface::nl_command_callback(const std_msgs::String::ConstPtr& msg){

    std::string cmd = msg->data;
    ROS_INFO("I heard [%s]",cmd.c_str());

    request.cmd = cmd;
    service_callback(request,response);


}

bool Cmd_interface::service_callback(peg_hole_kuka::String_cmd::Request& req,peg_hole_kuka::String_cmd::Response &res){

    std::string key_name = req.cmd;
    it = cmds.find(key_name);
    cmd_info& cmd_inf = it->second;


    if(it != cmds.end()){
        cmd_type cmd_t  = cmd_inf.c_type;
        std::string cmd = cmd_inf.cmd_name;

        switch(cmd_t){
        case cmd_type::ACTION:
        {
            return call_action(cmd);
            break;
        }
        case cmd_type::PEG_POLICY:
        {
            return call_peg_policy(cmd);
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
        return false;
    }else{

        res.res = "Cmd_interface::service_callback      Error no such label [" + key_name + "] defined! ";
        return false;
    }
}

bool Cmd_interface::call_peg_policy(const std::string& cmd){
    peg_hole_policy.request.cmd = cmd;
    if(!peg_policy_client.call(peg_hole_policy.request,peg_hole_policy.response)){
        std::cout<< peg_hole_policy.response.res << std::endl;
        return false;
    }else{
        std::cout<< peg_hole_policy.response.res << std::endl;
        return true;
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
    std::cout<< "print....." << std::endl;
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

