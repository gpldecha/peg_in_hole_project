#include "peg_hole_policy/policies/insert_peg.h"
#include <optitrack_rviz/type_conversion.h>

namespace ph_policy{


Insert_peg::Insert_peg()
{


    // load gmr model
    std::string path_gmm_parameters = "/home/guillaume/MatlabWorkSpace/peg_in_hole_RL/PolicyModelSaved/PolicyModel_txt/gmm_xsocket";

    // Load GMM parameters
    gmr_policy  = planners::GMR_EE_Planner(path_gmm_parameters);
    input.resize(3);
}

void Insert_peg::update(arma::colvec3& velocity,const arma::colvec3& mode_SF){

    input(0) = mode_SF(0);
    input(1) = mode_SF(1);
    input(2) = mode_SF(2);

    vel_direction.zeros();

    gmr_policy.gmr(input);
    gmr_policy.get_ee_linear_velocity(vel_direction);
    vel_direction = arma::normalise(vel_direction);

    if(!vel_direction.is_finite()){
        ROS_WARN_STREAM_THROTTLE(1.0, "vel_direction is not finite Insert_peg::get_linear_velocity");
    }

    velocity = vel_direction;
}


}
