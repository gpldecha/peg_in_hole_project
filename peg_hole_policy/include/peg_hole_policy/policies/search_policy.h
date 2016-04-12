#ifndef SEARCH_POLICY_PEG_H_
#define SEARCH_POLICY_PEG_H_

#include "peg_hole_policy/policies/base_find.h"
#include "peg_hole_policy/policies/get_back_on.h"
#include "robot_planners/gmmPlanner.h"
#include "exploration_planner/belief_gmm_planner.h"
#include <algorithm>

#include "peg_hole_policy/policies/state_machine.h"
#include "peg_hole_policy/policies/go_feature.h"
#include "peg_hole_policy/policies/force_control.h"
#include "peg_hole_policy/policies/insert_peg.h"

namespace ph_policy{

typedef enum actions{
    NONE,
    FIND_TABLE,
    GO_TO_EDGE,
    GET_BACK_ONTO_TABLE,
    GO_TO_SOCKET,
    FIND_SOCKET_HOLE,
    INSERT
}actions;

enum class SOCKET_POLICY{
    RIGHT_OUTER,
    LEFT_OUTER,
    TO_SOCKET,
    GO_TO_INSERT,
    INSERT
};

inline std::string actions2str(actions action){
    switch(action){
    case NONE:
        return "NONE";
    case FIND_TABLE:
        return "FIND_TABLE";
    case GO_TO_EDGE:
        return "GO_TO_EDGE";
    case GO_TO_SOCKET:
        return "GO_TO_SOCKET";
    case FIND_SOCKET_HOLE:
        return "FIND_SOCKET_HOLE";
    case INSERT:
        return "INSERT";
    case GET_BACK_ONTO_TABLE:
        return "GET_BACK_ONTO_TABLE";
    }
}

class Planning_stack{

public:

    Planning_stack();

    void update(const std::vector<STATES>& states);

    actions get_action() const;

    void reset();

    const void print(double seconds=0) const;

    static bool has_state(const STATES state, const std::vector<STATES>& states);

    void set_action(actions action);

private:

    void next_action();

private:

    std::vector<actions> stack;
    actions              current_action;
    std::size_t          index;
    bool                 bFrist;
};


class Search_policy{

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

    Search_policy(ros::NodeHandle& nh,
                  belief::Gmm_planner&  gmm_belief,
                  State_machine&        state_machine,
                  wobj::WrapObject&     wrap_object,
                  Peg_sensor_model&     peg_sensor_model);

    void get_velocity(tf::Vector3 &velocity,
                      tf::Quaternion& des_orient,
                      const tf::Vector3 &peg_origin,
                      const tf::Matrix3x3& peg_orient,
                      const arma::colvec& Y_c,
                      geometry_msgs::Wrench& wrench);

    void set_action(actions ctrl_policy);

    void set_socket_policy(SOCKET_POLICY socket_policy);

    void reset();

    void update_force_vis(const arma::colvec3 &force, const tf::Vector3 &position, const tf::Quaternion &orientation);

private:

    void get_target(FEATURE feature,arma::colvec3& target);

    void get_back_on_table();



public:

    State_machine&             state_machine;

private:

    belief::Gmm_planner&       gmm_belief;
    Insert_peg                 insert_peg;
    wobj::WBox&                socket_box;
    Peg_sensor_model&          peg_sensor_model;


    /// Socket policies
    planners::GMR_EE_Planner   right_circle_gmr, left_circle_gmr;
    planners::GMR_EE_Planner   gmm_socket;
    planners::GMAPlanner       gmm_search;
    SOCKET_POLICY              socket_policy;

    opti_rviz::Listener         peg_ee_listener;
   // tf::Matrix3x3               tf_peg_orient;
   // tf::Vector3                 tf_peg_origin;
    arma::colvec3               arma_peg_origin;


    Planning_stack              planning_stack;
    Get_back_on                 get_back_on;
    Go_freature                 go_edge,go_socket;


    tf::Quaternion               q_tmp;

    actions                 ctrl_policy;

    arma::colvec3           target_pos;
    arma::colvec3           mode_pos_WF,mode_pos_SF;
    arma::colvec3           arma_velocity;
    arma::colvec3           force;
    arma::colvec3           socket_pos_WF;

    Force_control           force_control;

};

}

#endif
