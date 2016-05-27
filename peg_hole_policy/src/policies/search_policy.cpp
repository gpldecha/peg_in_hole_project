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
    }else if(cmd == "forward"){
        policy = POLICY::FORWARD;
        forward_policy.reset();
        res    = "forward policy set!";
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
    }else if(cmd == "simple"){
        policy = POLICY::SIMPLE_POLICY;
        res = "simple policy set!";
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
    forward_policy.reset();
    close_to_socket = false;
    start_time      = ros::Time::now();

    pdf_norm = std::normal_distribution<double>(0,1);
}

Search_policy::POLICY Search_policy::get_policy(){
    return policy;
}

void Search_policy::get_velocity(tf::Vector3&           velocity,
                                 tf::Quaternion&        des_orient,
                                 const arma::colvec3&   peg_origin_WF,
                                 const tf::Matrix3x3&   peg_orient,
                                 const arma::colvec&    Y_c,
                                 const arma::colvec3&   force,
                                 const arma::colvec&    belief_state_WF,
                                 const arma::colvec&    belief_state_SF,
                                 const arma::colvec3&   socket_pos_WF,
                                 const arma::colvec3&   open_loop_x_origin_arma_WF)
{

    mls_WF(0) = belief_state_WF(0);
    mls_WF(1) = belief_state_WF(1);
    mls_WF(2) = belief_state_WF(2);
    mls_SF(0) = belief_state_SF(0);
    mls_SF(1) = belief_state_SF(1);
    mls_SF(2) = belief_state_SF(2);

    /** Y_c

        Y_mixed(0)  = Y_ft(0);                                                  // contact / no contact

        Y_mixed(1)  = Y_virtual(psm::Contact_distance_model::C_EDGE_DIST);      // Fx
        Y_mixed(2)  = Y_virtual(psm::Contact_distance_model::C_EDGE_V2);        // Fy
        Y_mixed(3)  = Y_virtual(psm::Contact_distance_model::C_EDGE_V3);        // Fz

        Y_mixed(4)  = Y_virtual(psm::Contact_distance_model::C_EDGE_LEFT);      // prob_left_edge
        Y_mixed(5)  = Y_virtual(psm::Contact_distance_model::C_EDGE_RIGHT);     // prob_right_edge
        Y_mixed(6)  = Y_virtual(psm::Contact_distance_model::C_EDGE_TOP);       // prob_up_edge
        Y_mixed(7)  = Y_virtual(psm::Contact_distance_model::C_EDGE_BOT);       // prob_down_edge
        Y_mixed(8)  = Y_virtual(psm::Contact_distance_model::C_SOCKET);         // Y_socket
        Y_mixed(9)  = Y_virtual(psm::Contact_distance_model::C_RING);
    **/


    target_WF    = socket_pos_WF;
   // target_WF(2) = target_WF(2) - 0.001;

    opti_rviz::debug::tf_debuf(target_WF,"TARGET_FINAL");

    search_time = (ros::Time::now() - start_time).toSec();

    if(Y_c(8) == 1)
    {
        in_socket=true;
    }else{
        in_socket=false;
    }

    bool debug = false;


    if(debug){
        std::cout<< "Search policy #1" << std::endl;
        std::cout<< "Y_c : " << Y_c.n_rows << " " << Y_c.n_cols << std::endl;
    }

    state_machine.update(arma_velocity,force,belief_state_WF,belief_state_SF,peg_orient,Y_c,get_back_on);
    state_machine.print(1);

    if(debug){ std::cout<< "Search policy #2" << std::endl;}

    const std::vector<STATES>& states  = state_machine.get_state();

    if(State_machine::has_state(STATES::CLOSE_SOCKET,states)){
        close_to_socket=true;
    }

    if(debug){ std::cout<< "Search policy #3" << std::endl;}

    des_orient  = des_orient_;

    if(debug){ std::cout<< "Search policy #4" << std::endl;}


    opti_rviz::type_conv::vec2tf(mls_WF,tf_mls_WF);
    peg_sensor_model.update_model(tf_mls_WF,peg_orient);
    peg_sensor_model.get_distance_features();

    if(debug){std::cout<< "Search policy #5" << std::endl;}

    arma_velocity.zeros();

    ROS_INFO_STREAM_THROTTLE(0.5,"search time: " << search_time);


    /// inside socket holes
   /* if(Y_c(8) == 1  && State_machine::has_state(STATES::LOW_UNCERTAINTY,states)){
        policy = POLICY::INSERT;
    }else if(dist_yz(target_WF,peg_origin_WF) < 0.01 && search_time > 30 && State_machine::has_state(STATES::LOW_UNCERTAINTY,states)){
        policy = POLICY::INSERT;
    }
*/

    switch(policy){
    case POLICY::SPECIALISED:
    {
        specialised_policy.update(arma_velocity,mls_WF,mls_SF,socket_pos_WF,peg_origin_WF,Y_c,states,insert_peg,get_back_on,force_control,peg_sensor_model);
        break;
    }
    case POLICY::GMM:
    {
        gmm.update(arma_velocity,belief_state_SF,states);


        break;
    }
    case POLICY::INSERT:
    {
        ROS_INFO_STREAM_THROTTLE(1.0,"[[POLICY::INSERT]]");
        arma_velocity.zeros();

        if(in_socket){
            arma_velocity.zeros();
            arma_velocity(0) = -0.01;

        }else if(dist_yz(target_WF,peg_origin_WF) < 0.001){

            // distribution(5.0,2.0);

            double                      noise_time;
            ros::Time                   noise_start_time;


            arma_velocity = target_WF - peg_origin_WF;

           // if(ros::Time::now() - noise_start_time).toSec() > 1){
          //      arma_velocity(1) = arma_velocity(1) + pdf_norm(generator);
          //      arma_velocity(2) = arma_velocity(2) + pdf_norm(generator);
          //  }



        }else{

            arma_velocity(1) = target_WF(1)  - peg_origin_WF(1);
            arma_velocity(2) = target_WF(2)  - peg_origin_WF(2);

        }

        if(arma::norm(arma_velocity) > 0.01)
        {
            arma_velocity = arma::normalise(arma_velocity);
            arma_velocity = 0.01 * arma_velocity;
        }

        break;
    }
    case POLICY::SIMPLE_POLICY:
    {
        simple_policies.update(arma_velocity,mls_WF,mls_SF,socket_pos_WF,peg_origin_WF,Y_c,states);
        break;
    }
    case POLICY::FORWARD:
    {
        ROS_INFO_STREAM_THROTTLE(1.0,"[[POLICY::FORWARD]]");
        forward_policy.update(arma_velocity,peg_origin_WF);
        break;
    }
    }

    /// Takes care if we are stuck at an edge
    if(policy != POLICY::INSERT && policy != POLICY::FORWARD){
        if(State_machine::has_state(STATES::STUCK_EDGE,states)){
            force_control.get_over_edge(arma_velocity,open_loop_x_origin_arma_WF,peg_origin_WF);
        }else{
            force_control.update_x(arma_velocity);
        }
    }

    force_control.force_safety(arma_velocity,8);

    if(policy != POLICY::INSERT && policy != POLICY::FORWARD){
        arma_velocity = arma::normalise(arma_velocity);
        if(!arma_velocity.is_finite()){
            ROS_WARN_THROTTLE(1.0,"arma_velocity is not finite() [Search_policy::get_velocity]!");
            arma_velocity.zeros();
        }
    }

    opti_rviz::type_conv::vec2tf(arma_velocity,velocity);

}

}
