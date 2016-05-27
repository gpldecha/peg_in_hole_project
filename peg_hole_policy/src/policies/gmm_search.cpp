#include "peg_hole_policy/policies/gmm_search.h"
#include <ros/ros.h>

namespace ph_policy{

GMM::GMM(SOCKET_TYPE socket_type):
  socket_type(socket_type)
{

    ROS_INFO("GMM policy Constructor [gmm_search.cpp]");
    std::string greedy_model = "/home/guillaume/MatlabWorkSpace/peg_in_hole_RL/gmr_policies/model/cpp/gmm_xsocket";
    std::string gmm_model    = "/home/guillaume/MatlabWorkSpace/peg_in_hole_RL/gmr_policies/model/cpp/gmm_xhu_socket";
    std::string qem_model    = "/home/guillaume/MatlabWorkSpace/peg_in_hole_RL/gmr_policies/model/cpp/qem_xhu_socket";

    ROS_INFO_STREAM("----> Initialise GMM_model <----");
    gma_gmm         =  planners::GMAPlanner(gmm_model);
    ROS_INFO_STREAM("----> Initialise QEM_model <----");
    gma_qem         =  planners::GMAPlanner(qem_model);
    ROS_INFO_STREAM("----> Initialise GREEDY_MODEL <----");
    gmr_greedy      =  planners::GMR_EE_Planner(greedy_model);
    ROS_INFO("Finished GMM policy Constructor [gmm_search.cpp]");
    velocity_tmp.zeros();
    b_air = true;

    belief_state_SF_tmp.resize(4);
}

void GMM::update(arma::colvec3& velocity,const arma::colvec& belief_state_SF,const std::vector<STATES>& states){


    belief_state_SF_tmp = belief_state_SF;

    if(socket_type == SOCKET_TYPE::TWO){
        belief_state_SF_tmp(0) = 0.015;
    }

    switch(type){
    case GMM_TYPE::GMM:
    {
        ROS_INFO_STREAM_THROTTLE(1.0,"[policy: GMM]");
        //belief_state_SF.print("belief_state_SF");
        gma_gmm.gmc(belief_state_SF_tmp,velocity_tmp);
        gma_gmm.get_ee_linear_velocity(velocity);
        break;
    }
    case GMM_TYPE::QEM:
    {
        ROS_INFO_STREAM_THROTTLE(1.0,"[policy: QEM]");
        gma_qem.gmc(belief_state_SF_tmp,velocity_tmp);
        gma_qem.get_ee_linear_velocity(velocity);
        break;
    }
    case GMM_TYPE::GREEDY:
    {
        ROS_INFO_STREAM_THROTTLE(1.0,"[policy: GREEDY]");
        gmr_greedy.gmr(belief_state_SF_tmp(arma::span(0,2)));
        gmr_greedy.get_ee_linear_velocity(velocity);
        break;
    }
    }

    if(State_machine::has_state(STATES::AIR_HIGH,states))
    {
        b_air = true;
    }

    if(!State_machine::has_state(STATES::AIR,states)){
        b_air = false;
    }

    if(b_air){
        velocity.zeros();
        velocity(0) = -1;
    }

    velocity     =  arma::normalise(velocity);
    velocity_tmp = velocity;
}

void GMM::set_gmm(GMM_TYPE type){
    this->type = type;
}


}
