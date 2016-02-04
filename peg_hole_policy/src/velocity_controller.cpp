#include "peg_hole_policy/velocity_controller.h"

namespace ph_policy{

Velocity_controller::Velocity_controller(ros::NodeHandle& nh){


    ctrl_type = CONSTANTE;

    Kp     = 6;
    Ki     = 1;
    Kd     = 2.0;
    Ki_max = 0.3;
    Ki_min = -0.3;

    pid.resize(3);

    for(std::size_t i = 0; i < 3;i++){
        pid[i].initPid(Kp,Ki,Kd,Ki_max,Ki_min);
    }

    C = 1.0;

    nd1 = ros::NodeHandle("gains");


    dynamic_server_gains_param.reset(new dynamic_reconfigure::Server< peg_hole_policy::gainsConfig>( nd1 ));
    dynamic_server_gains_param->setCallback( boost::bind(&Velocity_controller::gains_callback, this, _1, _2));

}


void Velocity_controller::update(tf::Vector3 &dx_cmd, const tf::Vector3 &dx_des, const tf::Vector3 &dx_current,const ros::Duration& ros_dt){


    if(ctrl_type == CONSTANTE)
    {
        dx_cmd = C * dx_des;
    }else{
        dx_cmd.setX(  pid[0].updatePid(dx_current.getX() - dx_des.getX(),ros_dt)  );
        dx_cmd.setY(  pid[1].updatePid(dx_current.getY() - dx_des.getY(),ros_dt)  );
        dx_cmd.setZ(  pid[1].updatePid(dx_current.getZ() - dx_des.getZ(),ros_dt)  );
    }

}

void Velocity_controller::gains_callback(peg_hole_policy::gainsConfig& config, uint32_t level){

    Kp      = config.Kp;
    Ki      = config.Ki;
    Kd      = config.Kd;
    Ki_max  = config.Ki_max;
    Ki_min  = config.Ki_min;

    for(std::size_t i = 0; i < pid.size();i++){
        pid[i].setGains(Kp,Ki,Kd,Ki_max,Ki_min);
    }

    C      = config.C;
}


}
