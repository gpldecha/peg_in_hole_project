#ifndef SPECIALISED_POLICY_PEG_H_
#define SPECIALISED_POLICY_PEG_H_

#include <string>
#include "robot_planners/gmmPlanner.h"
#include <armadillo>
#include <tf/LinearMath/Vector3.h>
#include <ros/ros.h>

#include "peg_hole_policy/stack_planner/state_machine.h"
#include "peg_hole_policy/stack_planner/planning_stack.h"

#include "peg_hole_policy/policies/go_feature.h"
#include "peg_hole_policy/policies/force_control.h"
#include "peg_hole_policy/policies/insert_peg.h"

namespace ph_policy{

class Specialised{

public:

    enum class SOCKET_POLICY{
        RIGHT_OUTER,
        LEFT_OUTER,
        TO_SOCKET,
        GO_TO_INSERT,
        INSERT
    };

public:

    enum class FEATURE{
        SOCKET,
        ML_EDGE,
        MR_EDGE,
        TL_CORNER,
        TR_CORNER,
        BL_CORNER,
        BR_CORNER
    };

public:

    Specialised(wobj::WrapObject& wrap_object);

    void reset();

    void update(arma::colvec3&              velocity,
                const arma::colvec3&        mls_WF,
                const arma::colvec3&        mls_SF,
                const arma::colvec3&        socket_pos_WF,
                const arma::colvec3&        peg_origin,
                const arma::colvec&         Y_c,
                const std::vector<STATES>&  states,
                Insert_peg&                 insert_peg,
                Get_back_on&                get_back_on,
                Force_control&              force_control,
                Peg_sensor_model&           peg_sensor_model);

    void set_socket_policy(SOCKET_POLICY socket_policy);

    std::string command(const std::string& cmd);

public:

    SOCKET_POLICY              socket_policy;

private:

    /// Socket policies
    planners::GMR_EE_Planner    right_circle_gmr, left_circle_gmr, gmm_socket;
    actions                     ctrl_policy;

    Planning_stack              planning_stack;
    Go_freature                 go_edge,go_socket;

    bool                        b_air;


};

}

#endif
