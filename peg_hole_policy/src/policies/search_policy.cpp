#include "peg_hole_policy/policies/search_policy.h"
#include "optitrack_rviz/type_conversion.h"
#include <optitrack_rviz/debug.h>

namespace ph_policy{

Search_policy::Search_policy(ros::NodeHandle&      nh,
                             Get_back_on&          get_back_on,
                             Specialised&          specialised_policy,
                             GMM&                   qem,
                             State_machine&        state_machine,
                             Peg_sensor_model&     peg_sensor_model):
    get_back_on(get_back_on),
    specialised_policy(specialised_policy),
    gmm(qem),
    state_machine(state_machine),
    force_control(nh),
    peg_sensor_model(peg_sensor_model)
{
    tf::Quaternion q_tmp;
    q_tmp.setRPY(0,-M_PI/2,0);
    des_orient_.setRPY(0,0,0);
    des_orient_ = q_tmp * des_orient_;

    reset();

}

void Search_policy::update_force_vis(const arma::colvec3& force,const tf::Vector3& position, const tf::Quaternion& orientation){
    force_control.update(force,position,orientation);
}

std::string Search_policy::command(const std::string& cmd,const std::vector<std::string>& args){
    std::string res;
    if(cmd == "special")
    {
        policy = POLICY::SPECIALISED;
        res = specialised_policy.command(args[0]);
    }else if(cmd == "greedy"){
        gmm.set_gmm(GMM::GMM_TYPE::GREEDY);
        policy = POLICY::GMM;
        res = "greedy policy set!";
    }else if(cmd == "gmm"){
        gmm.set_gmm(GMM::GMM_TYPE::GMM);
        policy = POLICY::GMM;
        res = "gmm policy set!";
    }else if(cmd == "qem"){
        gmm.set_gmm(GMM::GMM_TYPE::QEM);
        policy = POLICY::GMM;
        res = "qem policy set!";
    }else if(cmd == "insert"){
        policy = POLICY::INSERT;
        res = "insert policy set!";
    }else if(cmd == "reset"){
        reset();
        res = "reset [Search_policy]";
    }else{
        res = "no such cmd: " + cmd + " [Search_policy]";
    }
   return res;
}

void Search_policy::reset(){

    specialised_policy.reset();
    get_back_on.reset();
}

void Search_policy::get_velocity(tf::Vector3&           velocity,
                                 tf::Quaternion&        des_orient,
                                 const arma::colvec3&   peg_origin,
                                 const tf::Matrix3x3&   peg_orient,
                                 const arma::colvec&    Y_c,
                                 const arma::colvec3&   force,
                                 const arma::colvec&    belief_state_WF,
                                 const arma::colvec&    belief_state_SF,
                                 const arma::colvec3&    socket_pos_WF)
{

    mls_WF(0) = belief_state_WF(0);
    mls_WF(1) = belief_state_WF(1);
    mls_WF(2) = belief_state_WF(2);
    mls_SF(0) = belief_state_SF(0);
    mls_SF(1) = belief_state_SF(1);
    mls_SF(2) = belief_state_SF(2);


    state_machine.update(arma_velocity,force,belief_state_WF,belief_state_SF,peg_orient,Y_c,get_back_on);
    state_machine.print(1);

    const std::vector<STATES>& states  = state_machine.get_state();

    des_orient  = des_orient_;

    opti_rviz::type_conv::vec2tf(mls_WF,tf_mls_WF);
    peg_sensor_model.update_model(tf_mls_WF,peg_orient);
    peg_sensor_model.get_distance_features();

/*    opti_rviz::debug::tf_debuf<float>(peg_sensor_model.get_closet_point(contact_type::EDGE),"edge_projection");
    opti_rviz::debug::tf_debuf(mode_pos_WF,"mode_pos_WF");*/

    arma_velocity.zeros();

    switch(policy){
    case POLICY::SPECIALISED:
    {
        specialised_policy.update(arma_velocity,mls_WF,mls_SF,socket_pos_WF,peg_origin,Y_c,states,insert_peg,get_back_on,force_control,peg_sensor_model);
        if(specialised_policy.socket_policy == Specialised::SOCKET_POLICY::INSERT){policy = POLICY::INSERT;}
        break;
    }
    case POLICY::GMM:
    {
        gmm.update(arma_velocity,belief_state_SF);
        break;
    }
    case POLICY::INSERT:
    {
        ROS_INFO_STREAM_THROTTLE(1.0,"[[POLICY::INSERT]]");
        arma_velocity(0) = -1;
        arma_velocity(1) =  0;
        arma_velocity(2) =  0;
        break;
    }
    }

    ///  keep peg whithin socket region
    ROS_INFO_STREAM_THROTTLE(1.0,"socket_box.dist_edge:   " << peg_sensor_model.get_distance_edge() );
    if(peg_sensor_model.get_distance_edge() > 0.015){
        arma_velocity = (socket_pos_WF - mls_WF);
        arma_velocity = arma::normalise(arma_velocity);
    }

    /// Takes care if we are stuck at an edge
    if(policy != POLICY::INSERT){
        if(State_machine::has_state(STATES::STUCK_EDGE,states)){
            force_control.get_over_edge(arma_velocity);
        }else{
            force_control.update_x(arma_velocity);
        }
    }

    force_control.force_safety(arma_velocity,8);

    arma_velocity = arma::normalise(arma_velocity);
    if(!arma_velocity.is_finite()){
        ROS_WARN_THROTTLE(1.0,"arma_velocity is not finite() [Search_policy::get_velocity]!");
        arma_velocity.zeros();
    }
    opti_rviz::type_conv::vec2tf(arma_velocity,velocity);

}

}
