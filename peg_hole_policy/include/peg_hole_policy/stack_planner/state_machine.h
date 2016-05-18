#ifndef POLICIES_STATE_MACHINE_H_
#define POLICIES_STATE_MACHINE_H_

#include <armadillo>
#include <ros/ros.h>
#include <peg_sensor/peg_sensor_model/peg_sensor_model.h>
#include <optitrack_rviz/debug.h>
#include <peg_hole_policy/policies/get_back_on.h>
#include <Eigen/Eigen>
#include <geometry_msgs/Wrench.h>

namespace ph_policy{

enum class STATES{
    AIR,
    AIR_HIGH,
    TABLE,
    EDGE,
    SOCKET,
    CLOSE_EDGE,
    CLOSE_SOCKET,
    OFF_TABLE,
    GETTING_BACK_ON_TABLE,
    STUCK_EDGE,
    LOW_UNCERTAINTY,
    MEDIUM_UNCERTAINTY,
    HIGH_UNCERTAINTY,
    TOP_SOCKET,
    BOTTOM_SOCKET,
    LEFT_SOCKET,
    RIGHT_SOCKET,
    SOCKET_ENTRY,
    SLIGHTLY_IN,
    MODE_CLOSE_HOLE
};


inline std::string states2str(STATES state){
    switch(state)
    {
    case STATES::AIR:
        return "AIR";
    case STATES::AIR_HIGH:
        return "AIR_HIGH";
    case STATES::TABLE:
        return "TABLE";
    case STATES::EDGE:
        return "EDGE";
    case STATES::SOCKET:
        return "SOCKET";
    case STATES::CLOSE_EDGE:
        return "CLOSE_EDGE";
    case STATES::CLOSE_SOCKET:
        return "CLOSE_SOCKET";
    case STATES::OFF_TABLE:
        return "OFF_TABLE";
    case STATES::GETTING_BACK_ON_TABLE:
        return "GETTING_BACK_ON_TABLE";
    case STATES::STUCK_EDGE:
        return "STUCK_EDGE";
    case STATES::LOW_UNCERTAINTY:
        return "LOW_UNCERTAINTY";
    case STATES::MEDIUM_UNCERTAINTY:
        return "MEDIUM_UNCERTAINTY";
    case STATES::HIGH_UNCERTAINTY:
        return "HIGH_UNCERTAINTY";
    case STATES::TOP_SOCKET:
        return "TOP_SOCKET";
    case STATES::BOTTOM_SOCKET:
        return "BOTTOM_SOCKET";
    case STATES::LEFT_SOCKET:
        return "LEFT_SOCKET";
    case STATES::RIGHT_SOCKET:
        return "RIGHT_SOCKET";
    case STATES::SOCKET_ENTRY:
        return "SOCKET_ENTRY";
    case STATES::SLIGHTLY_IN:
        return "SLIGHTLY_IN";
    case STATES::MODE_CLOSE_HOLE:
        return "MODE_CLOSE_HOLE";

    }
}

class State_machine{

public:

    State_machine(Peg_sensor_model &peg_sensor_model);

    void update(const arma::colvec3& velocity,
                const arma::colvec3 &force,
                const arma::colvec &belief_state_WF,
                const arma::colvec& belief_state_SF,
                const tf::Matrix3x3& Rot,
                const arma::colvec& Y_c,
                const Get_back_on& get_back_on);

    const std::vector<STATES> &get_state() const;

    const void print(double seconds=0) const;

    static bool has_state(const STATES state,const std::vector<STATES>& states){
        return std::find(states.begin(),states.end(),state) != states.end();
    }

private:

    bool is_off_table(const arma::colvec3& mode_SF, const arma::colvec3& force, STATES uncertainty);

private:

    Peg_sensor_model    &peg_sensor_model;
    STATES state, state_tmp,tmp;
    std::vector<STATES> states;
    tf::Vector3         mode_tf;
    arma::colvec3       mode_SF,mode_WF;
    double              H, dist_edge;

private:

};

}


#endif
