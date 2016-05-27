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
#include "peg_hole_policy/policies/simple_policies.h"

#include <random>


namespace ph_policy{


class Search_policy{

public:

    enum class POLICY{SPECIALISED,GMM,INSERT,SIMPLE_POLICY,FORWARD};


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
                      const arma::colvec3&          socket_pos_WF,
                      const arma::colvec3&          open_loop_x_origin_arma_WF);

    POLICY get_policy();

    void reset();

    void update_force_vis(const arma::colvec3 &force, const tf::Vector3 &position, const tf::Quaternion &orientation);

    std::string command(const std::string& cmd, const std::vector<std::string> &args = std::vector<std::string>(0));

private:

    void get_back_on_table();

    inline double dist_yz(const arma::colvec3& v1, const arma::colvec3& v2){
        return std::sqrt( (v1(1) - v2(1)) * (v1(1) - v2(1)) + (v1(2) - v2(2)) * (v1(2) - v2(2))  );
    }



public:

    State_machine&             state_machine;

private:


    arma::colvec3               arma_velocity;
    arma::colvec3               mls_WF;
    arma::colvec3               mls_SF;
    arma::colvec3               target_WF;
    tf::Vector3                 tf_mls_WF;

    Insert_peg                 insert_peg;
    Peg_sensor_model&          peg_sensor_model;

    Specialised&                specialised_policy;
    ph_policy::GMM&             gmm;
    Get_back_on&                get_back_on;
    Simple_policies             simple_policies;
    Forward_insert              forward_policy;

    bool                        close_to_socket;
    bool                        in_socket;
    ros::Time                   start_time;
    double                      search_time;

    double                      noise_time;
    ros::Time                   noise_start_time;


    tf::Quaternion              des_orient_;

    POLICY                      policy;

    std::default_random_engine          generator;
    std::normal_distribution<double>    pdf_norm;


    Force_control           force_control;

};

}

#endif
