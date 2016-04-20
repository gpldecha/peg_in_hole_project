#ifndef GREEDY_POLICY_PEG_H_
#define GREEDY_POLICY_PEG_H_

#include <string>
#include "robot_planners/gmmPlanner.h"
#include <armadillo>
#include <tf/LinearMath/Vector3.h>

namespace ph_policy{

class Greedy{

public:

    Greedy(const std::string& path_model);

    void get_velocity(tf::Vector3 &velocity, const arma::colvec3 &ml_state);

private:

    planners::GMR_EE_Planner   greedy_gmm;
    arma::colvec3              direction;

};

}

#endif
