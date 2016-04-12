#include "peg_hole_policy/policies/go_feature.h"
#include <ros/ros.h>
#include <optitrack_rviz/debug.h>

namespace ph_policy{

Go_freature::Go_freature(wobj::WrapObject &wrap_object):
      wall_box(wrap_object.get_wbox("link_wall")),
      socket_box(wrap_object.get_wbox("wbox_socket"))
{

    //    tl,tp,bl,br

    w_tr = arma::conv_to<arma::colvec>::from(wall_box.corners[0].C);
    w_tl = arma::conv_to<arma::colvec>::from(wall_box.corners[1].C);
    w_br = arma::conv_to<arma::colvec>::from(wall_box.corners[4].C);
    w_bl = arma::conv_to<arma::colvec>::from(wall_box.corners[5].C);

    s_tr = arma::conv_to<arma::colvec>::from(socket_box.corners[0].C);
    s_br = arma::conv_to<arma::colvec>::from(socket_box.corners[1].C);
    s_tl = arma::conv_to<arma::colvec>::from(socket_box.corners[2].C);
    s_bl = arma::conv_to<arma::colvec>::from(socket_box.corners[3].C);

    w_ml = 0.5 * (w_tr - w_br) + w_br;
    w_mr = 0.5 * (w_tl - w_bl) + w_bl;
    w_mt = 0.5 * (w_tl - w_tr) + w_tr;
    w_mb = 0.5 * (w_bl - w_br) + w_br;

    s_ml = 0.5 * (s_tr - s_br) + s_br;
    s_mr = 0.5 * (s_tl - s_bl) + s_bl;
    s_mt = 0.5 * (s_tl - s_tr) + s_tr;
    s_mb = 0.5 * (s_bl - s_br) + s_br;

    reset();

    /*  pf::tf_debuf<float>(w_tl,"w_tl");
    pf::tf_debuf<float>(w_tr,"w_tr");
    pf::tf_debuf<float>(w_bl,"w_bl");
    pf::tf_debuf<float>(w_br,"w_br");

    pf::tf_debuf<float>(s_tl,"s_tl");
    pf::tf_debuf<float>(s_tr,"s_tr");
    pf::tf_debuf<float>(s_bl,"s_bl");
    pf::tf_debuf<float>(s_br,"s_br");

    pf::tf_debuf<float>(w_ml,"w_ml");
    pf::tf_debuf<float>(w_mr,"w_mr");
    pf::tf_debuf<float>(w_mt,"w_mt");
    pf::tf_debuf<float>(w_mb,"w_mb");

    pf::tf_debuf<float>(s_ml,"s_ml");
    pf::tf_debuf<float>(s_mr,"s_mr");
    pf::tf_debuf<float>(s_mt,"s_mt");
    pf::tf_debuf<float>(s_mb,"s_mb");*/

    bSetTarget = true;

}

Go_freature::STATUS Go_freature::get_status() const{
    return status;
}

void   Go_freature::set_status(STATUS status){
    this->status = status;
}

void Go_freature::reset(){
    status = STATUS::FINISHED;
}


void Go_freature::set_target(const arma::colvec3& pos_WF){
    target_pos = pos_WF;
    bSetTarget = true;
}


void Go_freature::update(arma::colvec3 &velocity,const arma::colvec3& mode_pos_WF){
    if(status == Go_freature::STATUS::FINISHED || bSetTarget)
    {
        status          = STATUS::RUNNING;
        target_velocity = target_pos  - mode_pos_WF;
        bSetTarget      = false;
    }

    velocity   = target_velocity;
    velocity   = arma::normalise(velocity);

    opti_rviz::debug::tf_debuf(target_pos,"go edge target_pos");

    if(!velocity.is_finite()){
        ROS_WARN_STREAM_THROTTLE(1.0,"!velocity.is_finite(): " << velocity(0) << " " << velocity(1) << " " << velocity(2) << " [Go_freature::update]");
        velocity.zeros();
    }
}


}
