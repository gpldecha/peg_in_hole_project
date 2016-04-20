#ifndef SEARCH_POLICY_PEG_H_
#define SEARCH_POLICY_PEG_H_

#include "peg_hole_policy/policies/base_find.h"
#include "peg_hole_policy/policies/get_back_on.h"
#include "robot_planners/gmmPlanner.h"
#include "exploration_planner/belief_gmm_planner.h"

#include <algorithm>
#include "peg_hole_policy/stack_planner/state_machine.h"
#include "peg_hole_policy/stack_planner/planning_stack.h"
#include <geometry_msgs/Wrench.h>
#include <peg_hole_policy/policies/greedy_policy.h>
#include "peg_hole_policy/policies/insert_peg.h"
#include "peg_hole_policy/policies/force_control.h"
#include "peg_hole_policy/policies/specialised_policy.h"
#include "peg_hole_policy/policies/gmm_search.h"

namespace ph_policy{


class Search_policy{

public:

    enum class POLICY{SPECIALISED,GMM,INSERT};


public:

    Search_policy(ros::NodeHandle&      nh,
                  Get_back_on&          get_back_on,
                  Specialised&          specialised_policy,
                  ph_policy::GMM&       gmm,
                  State_machine&        state_machine,
                  Peg_sensor_model&     peg_sensor_model);

    void get_velocity(tf::Vector3&                  velocity,
                      tf::Quaternion&               des_orient,
                      const arma::colvec3&          peg_origin,
                      const tf::Matrix3x3&          peg_orient,
                      const arma::colvec&           Y_c,
                      const arma::colvec3&          force,
                      const arma::colvec&           belief_state_WF,
                      const arma::colvec&           belief_state_SF,
                      const arma::colvec3&          socket_pos_WF);

    void reset();

    void update_force_vis(const arma::colvec3 &force, const tf::Vector3 &position, const tf::Quaternion &orientation);

    std::string command(const std::string& cmd, const std::vector<std::string> &args = std::vector<std::string>(0));

private:

    void get_back_on_table();



public:

    State_machine&             state_machine;

private:


    arma::colvec3               arma_velocity;
    arma::colvec3               mls_WF;
    arma::colvec3               mls_SF;
    tf::Vector3                 tf_mls_WF;

    Insert_peg                 insert_peg;
    Peg_sensor_model&          peg_sensor_model;

    Specialised&                specialised_policy;
    ph_policy::GMM&             gmm;
    Get_back_on&                get_back_on;


    tf::Quaternion              des_orient_;

    POLICY                      policy;



    Force_control           force_control;

};

}

#endif
