#include "peg_hole_kuka/cmd_interface.h"

Cmd_interface::Cmd_interface(ros::NodeHandle &nh,
                             const std::string& service_name,
                             const std::string& action_client_name,
                             const std::string& pf_client_name,
                             const std::string& exploration_client_name,
                             const std::string& peg_policy_client_name,
                             const std::string& voice_topic_name,
                             const std::string& record_topic_name)
{
    service                 = nh.advertiseService(service_name,&Cmd_interface::service_callback,this);


    action_client           = nh.serviceClient<lwr_ros_client::String_cmd>(action_client_name);

    particle_filter_client  = nh.serviceClient<particle_filter::String_cmd>(pf_client_name);
    exploration_client      = nh.serviceClient<exploration_planner::String_cmd>(exploration_client_name);
    peg_policy_client       = nh.serviceClient<peg_hole_policy::String_cmd>(peg_policy_client_name);
    netft_client            = nh.serviceClient<netft_rdt_driver::String_cmd>("/ft_sensor/bias_cmd");

    nl_subscriber           = nh.subscribe(voice_topic_name,1,&Cmd_interface::nl_command_callback,this);

    record_client           = nh.serviceClient<record_ros::String_cmd>(record_topic_name);

    init_commands();

}

void Cmd_interface::set_console(Console* console){
    this->console = console;
}

void Cmd_interface::init_commands(){

    cmds["plug_search"]         = cmd_info("plug_search",cmd_type::ACTION);

    cmds["stop"]                = cmd_info("stop",      cmd_type::PEG_POLICY);
    cmds["go_table"]            = cmd_info("go_table",  cmd_type::PEG_POLICY);
    cmds["go_socket"]           = cmd_info("go_socket", cmd_type::PEG_POLICY);
    cmds["insert"]              = cmd_info("insert",    cmd_type::PEG_POLICY);
    cmds["pause"]               = cmd_info("pause",     cmd_type::PEG_POLICY);
    cmds["grav_comp"]           = cmd_info("grav_comp", cmd_type::PEG_POLICY);
    cmds["go_front"]            = cmd_info("go_front",  cmd_type::PEG_POLICY);
    cmds["go_left"]             = cmd_info("go_left",   cmd_type::PEG_POLICY);
    cmds["home"]                = cmd_info("home",      cmd_type::PEG_POLICY);
    cmds["bel_reset"]           = cmd_info("bel_reset", cmd_type::PEG_POLICY);


    cmds["special"]             = cmd_info("special",   cmd_type::PEG_POLICY);
    cmds["greedy"]              = cmd_info("greedy",    cmd_type::PEG_POLICY);
    cmds["gmm"]                 = cmd_info("gmm",       cmd_type::PEG_POLICY);
    cmds["qem"]                 = cmd_info("qem",       cmd_type::PEG_POLICY);
    cmds["simple"]              = cmd_info("simple",    cmd_type::PEG_POLICY);
    cmds["forward"]             = cmd_info("forward",    cmd_type::PEG_POLICY);

    // for demos
    cmds["gmm_1"]               = cmd_info("gmm_1",   cmd_type::PEG_POLICY);
    cmds["gmm_2"]               = cmd_info("gmm_2",   cmd_type::PEG_POLICY);
    cmds["gmm_3"]               = cmd_info("gmm_3",   cmd_type::PEG_POLICY);


    cmds["open_loop"]           = cmd_info("open_loop",     cmd_type::PEG_POLICY);
    cmds["passive_ds"]          = cmd_info("passive_ds",    cmd_type::PEG_POLICY);
    cmds["go_peg_right"]        = cmd_info("go_peg_right",  cmd_type::PEG_POLICY);
    cmds["disconnect"]          = cmd_info("disconnect",    cmd_type::PEG_POLICY);

    cmds["record"]              = cmd_info("record",    cmd_type::RECORD);
    cmds["bias"]                = cmd_info("bias",      cmd_type::FT);

    cmds["pf_reset"]            = cmd_info("pf reset",  cmd_type::PF);
    cmds["pf_start"]            = cmd_info("pf start",  cmd_type::PF);
    cmds["print"]               = cmd_info("print",     cmd_type::UTILITY);
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

        console->Print("cmd: " + cmd);

        switch(cmd_t){
        case cmd_type::ACTION:
        {
            return call_action(cmd);
            break;
        }
        case cmd_type::PEG_POLICY:
        {
            return call_peg_policy(cmd,res.res);
            break;
        }
        case cmd_type::PF:
        {
            return call_particle_filter(cmd);
            break;
        }
        case cmd_type::FT:
        {
            call_netft(cmd,res.res);
            break;
        }
        case cmd_type::PLANNER:
            return call_planner(cmd);
            break;
        case cmd_type::UTILITY:
        {
            return call_utility(cmd,res.res);
            break;
        }
        case cmd_type::RECORD:
        {
            return call_record(cmd);
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

bool Cmd_interface::call_netft(const std::string& cmd, std::string& res){
    netft_cmd.request.cmd = cmd;
    if(!netft_client.call(netft_cmd.request,netft_cmd.response)){
        res = netft_cmd.response.res;
        return false;
    }else{
        res = netft_cmd.response.res;
        return true;
    }
}

bool Cmd_interface::call_peg_policy(const std::string& cmd, std::string& res){
    peg_hole_policy.request.cmd = cmd;
    if(!peg_policy_client.call(peg_hole_policy.request,peg_hole_policy.response)){
        res = peg_hole_policy.response.res;
        console->Print(peg_hole_policy.response.res);

        return false;
    }else{
        res = peg_hole_policy.response.res;
        console->Print(peg_hole_policy.response.res);
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

bool Cmd_interface::call_record(const std::string& cmd){
    if(cmd == "record"){
        record_cmd.request.cmd  = "record";
        record_cmd.response.res = "None";
        record_client.call(record_cmd);
        console->Print(" " + record_cmd.response.res);
        return true;
    }else{
        ROS_ERROR_STREAM("no such cmd [" << cmd << "]");
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

bool Cmd_interface::call_utility(const std::string& cmd, std::string& res){
    res =  "print.....\n";
    if(cmd == "print"){
        std::cout<< std::endl;
        res = res + " === cmd interface === \n";
        for(it = cmds.begin(); it != cmds.end();it++){
        //std::cout<< "  " << it->first << std::endl;
        res = res + it->first + "\n";
        }
        return true;
    }else{
        return false;
    }

}

