#include "peg_hole_policy/policies/simple_policies.h"
#include "optitrack_rviz/debug.h"

namespace ph_policy{


Simple_policies::Simple_policies(){

    b_air = false;

}



void  Simple_policies::update(arma::colvec3&                 velocity,
                              const arma::colvec3&           mls_WF,
                              const arma::colvec3&           mls_SF,
                              const arma::colvec3&           socket_pos_WF,
                              const arma::colvec3&           peg_origin,
                              const arma::colvec&            Y_c,
                              const std::vector<STATES>&     states){


    target = socket_pos_WF;
    target(0) = target(0) - 0.03;
    target(1) = target(1) - 0.08;



    opti_rviz::debug::tf_debuf(target,"TARGET");

    velocity = target - peg_origin;



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




Forward_insert::Forward_insert(){
    reset();
}

void Forward_insert::reset(){
    bFirst = true;
}

void Forward_insert::update(arma::colvec3& velocity,const arma::colvec3& peg_origin){

    if(bFirst)
    {
        target_x = peg_origin;
        target_x(0) = target_x(0) - 0.03;
        bFirst = false;
    }

    opti_rviz::debug::tf_debuf(target_x,"TARGET");

    velocity = target_x - peg_origin;
    velocity = arma::normalise(velocity);
}




}
