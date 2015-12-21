#ifndef PEG_INSERT_PEG_H_
#define PEG_INSERT_PEG_H_

#include "peg_hole_policy/policies/base_find.h"
#include "optitrack_rviz/listener.h"
#include "robot_planners/gmmPlanner.h"


namespace ph_policy{

class Insert_peg : public Base_find{

public:

    Insert_peg(ros::NodeHandle& nh,const std::string& path_sensor_model,const std::string& fixed_frame);

    virtual void get_linear_velocity(tf::Vector3& velocity,const tf::Vector3& peg_origin);

private:

    tf::Matrix3x3   Rt;
    tf::Vector3     T;
    tf::Vector3     pos_fr_socket; /// position of peg in the frame of reference of the socket

    planners::GMR_EE_Planner gmr_policy;
    arma::colvec  input;
    arma::colvec3 vel_direction;
};

}

#endif




