#include "peg_hole_policy/policies/insert_peg.h"
#include <optitrack_rviz/type_conversion.h>

namespace ph_policy{


Insert_peg::Insert_peg()
{

    tf::StampedTransform transform;
    opti_rviz::Listener::get_tf_once("world","link_socket",transform);
    Rt.setRotation(transform.getRotation());
    Rt = Rt.transpose();
    T = transform.getOrigin();

    // load gmr model
    std::string path_gmm_parameters = "/home/guillaume/MatlabWorkSpace/peg_in_hole_RL/PolicyModelSaved/PolicyModel_txt/gmm_xsocket";

    // Load GMM parameters

    gmr_policy  = planners::GMR_EE_Planner(path_gmm_parameters);

    std::cout<< "after loading gmr_model" << std::endl;
    input.resize(3);

    std::cout<< "=== Test GMR Policy === " << std::endl;
    input(0) = 0.06;
    input(1) = 0.04;
    input(2) = 0;

    gmr_policy.gmr(input);
    gmr_policy.get_ee_linear_velocity(vel_direction);

}

void Insert_peg::get_linear_velocity(arma::colvec3& velocity,const arma::colvec3& mode_WF){

    opti_rviz::type_conv::vec2tf(mode_WF,tmp);
    pos_fr_socket = Rt * (tmp - T);

    input(0) = pos_fr_socket.getX();
    input(1) = pos_fr_socket.getY();
    input(2) = pos_fr_socket.getZ();
    gmr_policy.gmr(input);

    vel_direction.zeros();
    gmr_policy.get_ee_linear_velocity(vel_direction);
    vel_direction = arma::normalise(vel_direction);

    if(!vel_direction.is_finite()){
        ROS_WARN_STREAM_THROTTLE(1.0, "vel_direction is not finite Insert_peg::get_linear_velocity");
    }

    velocity = vel_direction;
}


}
