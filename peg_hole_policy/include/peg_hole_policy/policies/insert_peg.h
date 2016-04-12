#ifndef PEG_INSERT_PEG_H_
#define PEG_INSERT_PEG_H_

#include "optitrack_rviz/listener.h"
#include "robot_planners/gmmPlanner.h"

namespace ph_policy{

class Insert_peg{

public:

    Insert_peg();

    void get_linear_velocity(arma::colvec3 &velocity,const arma::colvec3& mode_WF);

private:

    tf::Matrix3x3   Rt;
    tf::Vector3     T, tmp;
    tf::Vector3     pos_fr_socket; /// position of peg in the frame of reference of the socket

    planners::GMR_EE_Planner gmr_policy;
    arma::colvec  input;
    arma::colvec3 vel_direction;

};

}

#endif




