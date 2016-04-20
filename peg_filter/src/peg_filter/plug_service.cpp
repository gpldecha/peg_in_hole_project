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

void Plug_service::set_initilisation_function(const Initialise* initialise_f){
    this->initialise_f = initialise_f;
}

bool Plug_service::callback(particle_filter::String_cmd::Request& req,particle_filter::String_cmd::Response& res){

    if( (cmds.count(req.cmd))){
       switch(cmds[req.cmd])
       {
        case RESET:
       {

           ROS_INFO("Rest [Plug_service]");
           tf::StampedTransform transform;
           opti_rviz::Listener::get_tf_once("world","lwr_peg_link",transform);
           arma::colvec3 peg_position = {{transform.getOrigin().x(),
                                          transform.getOrigin().y(),
                                          transform.getOrigin().z()}};

           if(initialise_f != NULL){
            (*initialise_f)(peg_position(0),peg_position(1),peg_position(2));
           }else{
               ROS_ERROR("initialise_f == NULL [plug_service.cpp]");
           }

           ROS_INFO("Rest finished [Plug_service]");

         /*  peg_position.print("peg_position");
           plug_pf_manager.initialise_prior_pdf(peg_position);
           res.res = "reset particle_filter (" + boost::lexical_cast<std::string>(peg_position(0)) + " "
                                               + boost::lexical_cast<std::string>(peg_position(1)) + " "
                                               + boost::lexical_cast<std::string>(peg_position(2)) + ")";*/
           break;
       }
       case START:
       {
       /*     plug_pf_manager.bUpdate = !plug_pf_manager.bUpdate;
            if(plug_pf_manager.bUpdate){
                res.res = "start updating the particle filter";
            }else{
                res.res = "stop updating the particle filter";
            }
            break;*/
       }
       }
    }else{
        res.res = "NO SUCH COMMAND: " + req.cmd + "!";
        return false;
    }
    return true;
}


}
