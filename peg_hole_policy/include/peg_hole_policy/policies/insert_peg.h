#ifndef PEG_INSERT_PEG_H_
#define PEG_INSERT_PEG_H_

#include "optitrack_rviz/listener.h"
#include "robot_planners/gmmPlanner.h"

namespace ph_policy{

class Insert_peg{

public:

    Insert_peg();

    void update(arma::colvec3 &velocity,const arma::colvec3& mode_WF);

private:

    planners::GMR_EE_Planner gmr_policy;
    arma::colvec  input;
    arma::colvec3 vel_direction;

};

}

#endif




