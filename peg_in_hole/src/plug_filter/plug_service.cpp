#include <plug_filter/plug_service.h>

namespace plugfilter{

Plug_service::Plug_service(ros::NodeHandle& node,Plug_pf_manager& plug_pf_manager)
    :plug_pf_manager(plug_pf_manager)
{
    service = node.advertiseService("pf_service", &Plug_service::callback,this);

    cmds["reset"] = RESET;
    cmds["beta"]  = BETA;
    cmds["noise"] = NOISE;
    cmds["start"] = START;

}

bool Plug_service::callback(particle_filter::String_cmd::Request& req,particle_filter::String_cmd::Response& res){

    if( (cmds.count(req.cmd))){
       switch(cmds[req.cmd])
       {
        case RESET:
       {
           plug_pf_manager.initialise_prior_pdf();
           res.res = "reset particle_filter";
           break;
       }
       case START:
       {
            plug_pf_manager.bUpdate = !plug_pf_manager.bUpdate;
            if(plug_pf_manager.bUpdate){
                res.res = "start updating the particle filter";
            }else{
                res.res = "stop updating the particle filter";
            }
            break;
       }
       }
    }else{
        res.res = "NO SUCH COMMAND: " + req.cmd + "!";
        return false;
    }
    return true;
}


}
