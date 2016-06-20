#include "peg_hole_policy/policies/demo_policies.h"


namespace ph_policy{

Demo_policies::Demo_policies(wobj::WrapObject &wrap_object, SOCKET_TYPE socket_type, GMM &gmm):
gmm(gmm)
{


    spolicy_one.reset(   new ph_policy::Specialised(wrap_object,socket_type)  );
    spolicy_two.reset(   new ph_policy::Specialised(wrap_object,socket_type)  );
    spolicy_three.reset(   new ph_policy::Specialised(wrap_object,socket_type)  );


    reset();

}


void Demo_policies::set_demo(DEMO_TYPE demo_type){
    this->demo_type = demo_type;
}

void Demo_policies::reset(){
    spolicy_one->reset();
    spolicy_two->reset();
    spolicy_three->reset();
    forward_policy.reset();

    spolicy_one->set_socket_policy(Specialised::SOCKET_POLICY::LEFT_OUTER);
    spolicy_two->set_socket_policy(Specialised::SOCKET_POLICY::RIGHT_OUTER);

    demo_type  = DEMO_TYPE::DEMO_1;
    policy_type = POLICY_TYPE::DEMO;


    gmm.set_gmm(ph_policy::GMM::GMM_TYPE::GREEDY);

}


void Demo_policies::update(arma::colvec3&         velocity,
                           const arma::colvec3&        mls_WF,
                           const arma::colvec3&        mls_SF,
                           const arma::colvec3&        socket_pos_WF,
                           const arma::colvec3&        peg_origin_WF,
                           const arma::colvec&         Y_c,
                           const std::vector<STATES>&  states,
                           Insert_peg&                 insert_peg,
                           Get_back_on&                get_back_on,
                           Force_control&              force_control,
                           Peg_sensor_model&           peg_sensor_model,
                           const arma::colvec&    belief_state_SF,
                           const arma::colvec3&   open_loop_x_origin_arma_WF)
{

    if(Y_c(8) == 1)
    {
       // policy_type = POLICY_TYPE::INSERT;
        in_socket=false;
    }else{
        in_socket=false;
    }


    if(policy_type == POLICY_TYPE::DEMO){

        switch(demo_type){
        case DEMO_TYPE::DEMO_1:
        {
            spolicy_one->update(velocity,mls_WF,mls_SF,socket_pos_WF,peg_origin_WF,Y_c,states,insert_peg,get_back_on,force_control,peg_sensor_model);
            break;
        }
        case DEMO_TYPE::DEMO_2:
        {
            spolicy_two->update(velocity,mls_WF,mls_SF,socket_pos_WF,peg_origin_WF,Y_c,states,insert_peg,get_back_on,force_control,peg_sensor_model);
            break;
        }
        case DEMO_TYPE::DEMO_3:
        {
            spolicy_three->update(velocity,mls_WF,mls_SF,socket_pos_WF,peg_origin_WF,Y_c,states,insert_peg,get_back_on,force_control,peg_sensor_model);
            break;
        }
        }

        if(Planning_stack::has_state(STATES::LOW_UNCERTAINTY,states)){
            policy_type = POLICY_TYPE::GREEDY;
            ROS_WARN_STREAM("SWITCHING TO GREEDY POLICY");
        }

    }else if(policy_type == POLICY_TYPE::GREEDY){
        ROS_WARN_STREAM_THROTTLE(1.0," == GREEDY POLICY");

        gmm.update(velocity,belief_state_SF,states);

        if(Planning_stack::has_state(STATES::SOCKET_ENTRY,states)){
            policy_type = POLICY_TYPE::INSERT;
            ROS_WARN_STREAM("SWITCHING TO INSERT POLICY");
        }

    }else if(policy_type == POLICY_TYPE::INSERT){
        ROS_WARN_STREAM_THROTTLE(1.0," == INSERT POLICY");

        velocity.zeros();

        target_WF    = socket_pos_WF;

        if( dist_yz(target_WF,peg_origin_WF) < 0.001){
            velocity = target_WF - peg_origin_WF;
        }else{

            velocity(1) = target_WF(1)  - peg_origin_WF(1);
            velocity(2) = target_WF(2)  - peg_origin_WF(2);
        }

        if(arma::norm(velocity) > 0.02)
        {
            velocity = arma::normalise(velocity);
            velocity = 0.02 * velocity;
        }

    }else if(policy_type == POLICY_TYPE::FORWARD){
        ROS_WARN_STREAM_THROTTLE(1.0,"[[POLICY::FORWARD]]");
        forward_policy.update(velocity,peg_origin_WF);
    }



    /// Takes care if we are stuck at an edge
    if(policy_type != POLICY_TYPE::INSERT && policy_type != POLICY_TYPE::FORWARD){
        if(State_machine::has_state(STATES::STUCK_EDGE,states)){
            force_control.get_over_edge(velocity,open_loop_x_origin_arma_WF,peg_origin_WF);
        }else{
            force_control.update_x(velocity);
        }
    }

    force_control.force_safety(velocity,8);

    if(policy_type != POLICY_TYPE::INSERT && policy_type != POLICY_TYPE::FORWARD){
        velocity = arma::normalise(velocity);
        if(!velocity.is_finite()){
            ROS_WARN_THROTTLE(1.0,"arma_velocity is not finite() [Search_policy::get_velocity]!");
            velocity.zeros();
        }
    }
}


}

