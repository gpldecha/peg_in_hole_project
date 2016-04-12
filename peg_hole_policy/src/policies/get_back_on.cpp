#include "peg_hole_policy/policies/get_back_on.h"
#include <optitrack_rviz/type_conversion.h>
#include <optitrack_rviz/debug.h>

namespace ph_policy{


Get_back_on::Get_back_on(wobj::WBox &wall_box):
    wall_box(wall_box)
{
    reset();

    tf::StampedTransform transform;
    opti_rviz::Listener::get_tf_once("world","link_socket",transform);
    opti_rviz::type_conv::tf2vec(transform.getOrigin(),middle);

}

void Get_back_on::update(arma::colvec3 &velocity, const arma::colvec3 &mode_pos_SF, const arma::colvec3& mode_pos_WF, const arma::colvec &Y_c){
    if(status == STATUS::FINISHED){
        status = STATUS::RUNNING;
        std::cout<< "first step get way poinst" << std::endl;
        get_way_pts(mode_pos_WF);
        index = 0;
        target = wpts[index];
        std::cout<< "first target set" << std::endl;
    }

    planner(mode_pos_SF,mode_pos_WF,Y_c);

    velocity = target - mode_pos_WF;
    velocity = arma::normalise(velocity);
    if(!velocity.is_finite()){
        ROS_WARN_STREAM_THROTTLE(1.0,"!velocity.is_finite(): " << velocity(0) << " " << velocity(1) << " " << velocity(2) );
        velocity.zeros();
    }
}

Get_back_on::STATUS Get_back_on::get_status() const{
    return status;
}

void Get_back_on::reset(){
    status = STATUS::FINISHED;
    wpts.resize(3);
    index = 0;
}

void Get_back_on::set_status(STATUS status){
    this->status = status;
}

void Get_back_on::planner(const arma::colvec3& mode_pos_SF,const arma::colvec3& mode_pos_WF,const arma::colvec& Y_c, double dist_threashod){

    if(index == 0)
    {
        if(mode_pos_SF(0) > 0.01){
            index++;
        }
    }else if(index == 1){
        if(arma::norm(mode_pos_WF - target) <= dist_threashod)
        {
            index++;
        }
    }else{

    }
    if(index >= wpts.size()){
        index  = wpts.size()-1;
    }
    if(index == wpts.size()-1 && Y_c(0) >= 1)
    {
        status = STATUS::FINISHED;
    }

    target = wpts[index];

}

void Get_back_on::get_way_pts(const arma::colvec3& mode_pos){
    P = arma::conv_to<arma::fcolvec>::from(mode_pos);
    wall_box.distance_to_features(P);
    edge_P  = arma::conv_to<arma::colvec>::from(wall_box.get_edge_projection());

    wpts[0]     = edge_P;
    wpts[0](0)  = wpts[0](0) + 0.04;

    wpts[1]  = 0.06 * (middle - wpts[0]) + wpts[0];

    wpts[2]    = wpts[1];
    wpts[2](0) = wpts[2](0) - 0.04;


}

}
