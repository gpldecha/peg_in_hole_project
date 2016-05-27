#include "peg_hole_policy/policies/force_control.h"
#include "optitrack_rviz/type_conversion.h"
#include "ros/ros.h"
#include "optitrack_rviz/debug.h"
#include "optitrack_rviz/listener.h"

namespace ph_policy{



GetOverObstacle::GetOverObstacle(){

}

void GetOverObstacle::get_over_it(arma::colvec3& velocity, const arma::colvec3& open_x_position_WF, const arma::colvec3 &peg_origin_WF){

    // run this when stuck at an edge


    target_WF = peg_origin_WF;
    target_WF(0) = target_WF(0) + 0.02;


}



Force_control::Force_control(ros::NodeHandle& nh)
    :vis_cylinder(nh,"force_profile"),
      vis_vector(nh,"force")
{

     bFirst = true;
     force_cylinders.resize(3);

     std::cout<< "Force_control vis_vector initialise" << std::endl;

     force_vector.resize(1);
     force_vector[0].set_rgba(0.9,0,0,1);
     force_vector[0].set_scale(0.005,0.01,0.01);

     vis_vector.initialise("world",force_vector);
     max_N = 6;
}

void Force_control::update(const arma::colvec3& force,const tf::Vector3& ee_position, const tf::Quaternion& ee_orientation){
    if(bFirst){
        force_cylinders[0].set_pos(ee_position,ee_orientation);
        force_cylinders[0].set_scale(0.005,0.005,0.05);

        force_cylinders[1].set_pos(ee_position,ee_orientation);
        force_cylinders[1].set_scale(0.005,0.005,0.05);
        force_cylinders[1].set_rgba(0,1,0,1);

        force_cylinders[2].set_pos(ee_position,ee_orientation);
        force_cylinders[2].set_scale(0.005,0.005,0.05);
        force_cylinders[2].set_rgba(0,0,1,1);

        vis_cylinder.initialise("world",force_cylinders);
        bFirst = false;
    }
    F(2) = force(0);
    F(1) = force(1);
    F(0) = force(2);
    F    = -F;
    F_display   = F;
    if(F_display(0) > max_N){F_display(0) = max_N;}
    if(F_display(1) > max_N){F_display(1) = max_N;}
    if(F_display(2) > max_N){F_display(2) = max_N;}
    if(F_display(0) < -max_N){F_display(0) = -max_N;}
    if(F_display(1) < -max_N){F_display(1) = -max_N;}
    if(F_display(2) < -max_N){F_display(2) = -max_N;}

    F_n       = (F_display / max_N);
    F_display =  0.05 * (F_display / max_N);

   // ROS_INFO_STREAM_THROTTLE(1.0,"F_n: " << F_n(0) << " " << F_n(1) << " " << F_n(2));

    q_tmp.setRPY(0,-M_PI/2,0);
    force_cylinders[0].set_pos(ee_position,q_tmp);
    force_cylinders[0].set_scale(0.005,0.005,F_display(0));
    force_cylinders[0].set_FR_tip();

    q_tmp.setEuler(-M_PI/2.0,-M_PI/2.0,0);
    force_cylinders[1].set_pos(ee_position,q_tmp);
    force_cylinders[1].set_scale(0.005,0.005,F_display(1));
    force_cylinders[1].set_FR_tip();

    q_tmp.setEuler(0,0,0);
    force_cylinders[2].set_pos(ee_position,q_tmp);
    force_cylinders[2].set_scale(0.005,0.005,F_display(2));
    force_cylinders[2].set_FR_tip();

    opti_rviz::type_conv::vec2tf(F_display,vel_tmp);
    force_vector[0].set_pos_dir(ee_position,vel_tmp);

    vis_cylinder.update(force_cylinders);
    vis_cylinder.publish();

    vis_vector.update(force_vector);
    vis_vector.publish();

}

void Force_control::get_over_edge(arma::colvec3& velocity,const arma::colvec3& open_x_position, const arma::colvec3& peg_origin){

    ROS_WARN_STREAM_THROTTLE(1.0,"GET_OVER_EDGE");

    if(arma::norm(open_x_position - peg_origin) > 0.02)
    {
       velocity.zeros();
       velocity(0) = 1;
       opti_rviz::type_conv::vec2tf(velocity,vel_tmp);
    }else{

        double fac_y =  -20 * F_n(1);
        double fac_z =   20 * F_n(2);

        if(fac_y > 1){fac_y = 1;}
        if(fac_z > 1){fac_z = 1;}
        if(fac_y < -1){fac_y = -1;}
        if(fac_z < -1){fac_z = -1;}

        Rz.setRPY(0,    0,  fac_y * (M_PI/2.0) );
        Ry.setRPY(0,    fac_z * (M_PI/2.0),   0 );

        opti_rviz::type_conv::vec2tf(velocity,vel_tmp);
        vel_tmp     =  Ry * Rz * vel_tmp;
    }

    opti_rviz::type_conv::tf2vec(vel_tmp,velocity);
    velocity    = arma::normalise(velocity);
}

void Force_control::regulise_force(arma::colvec3& velocity, double max_force){
    if(std::fabs(F(1)) > max_force ){
        velocity(1) =  0.1 * -sign( velocity(1) ) * scale(std::fabs(F(1)),max_force);
        ROS_WARN_STREAM_THROTTLE(0.5,"Force limit reached in Y");
    }
    if(std::fabs(F(2)) > max_force){
        velocity(2) =  0.1 * -sign( velocity(2) ) * scale(std::fabs(F(2)),max_force);
        ROS_WARN_STREAM_THROTTLE(0.5,"Force limit reached in Z");
    }
}

void Force_control::force_safety(arma::colvec3& velocity, double max_force){
    if(arma::norm(F) > max_force){
        velocity.zeros();
        ROS_WARN_STREAM_THROTTLE(1.0,"MAX force reached: " << arma::norm(F) << "  [force_control.cpp]");
    }
}

double Force_control::quadratic(double x){
   return 1.0 - ( x - 1.0) * (x - 1.0);
}

void Force_control::update_x(arma::colvec3 &velocity){
    velocity(0) = 0.2/6 * (std::fabs(F(0)) - 3);
    if(velocity(0) > 0.1){
        velocity(0)= 0.1;
    }
    if(velocity(0)< -0.1)
    {
        velocity(0) = -0.1;
    }
    velocity = arma::normalise(velocity);
}




}
