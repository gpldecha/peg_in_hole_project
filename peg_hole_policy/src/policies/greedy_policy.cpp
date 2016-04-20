#include "peg_hole_policy/policies/greedy_policy.h"
#include <optitrack_rviz/type_conversion.h>

namespace ph_policy{

Greedy::Greedy(const std::string& path_model){
    greedy_gmm = planners::GMR_EE_Planner(path_model);
}

void Greedy::get_velocity(tf::Vector3 &velocity, const arma::colvec3 &ml_state){

    greedy_gmm.gmr(ml_state);
    greedy_gmm.get_ee_linear_velocity(direction);
    direction = arma::normalise(direction);
    opti_rviz::type_conv::vec2tf(direction,velocity);
}



}
