#include <peg_filter/plug_service.h>
#include <boost/lexical_cast.hpp>

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
           // QUICK HACK FOR peg demo
           tf::StampedTransform transform;
           opti_rviz::Listener::get_tf_once("world_frame","peg_link",transform,10);
           arma::colvec3 peg_position = {{transform.getOrigin().x(),
                                          transform.getOrigin().y(),
                                          transform.getOrigin().z()}};

           peg_position.print("peg_position");
           plug_pf_manager.initialise_prior_pdf(peg_position);
           res.res = "reset particle_filter (" + boost::lexical_cast<std::string>(peg_position(0)) + " "
                                               + boost::lexical_cast<std::string>(peg_position(1)) + " "
                                               + boost::lexical_cast<std::string>(peg_position(2)) + ")";
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
