#include "peg_hole_policy/state_machine.h"

namespace ph_policy{

State_machine::State_machine(ros::NodeHandle &nh,const std::string& y_topic):
peg_sensor_listener(nh,y_topic)
{


    state = AIR;

}

void State_machine::update(Eigen::Vector3d& Y_c, geometry_msgs::Wrench &wrench){


    if(Y_c(0) >= 0.8)
    {
        tmp = CONTACT;
    }
    if(Y_c(1) >= 0.8)
    {
        tmp = EDGE;
    }
    if(Y_c(0) < 0.8)
    {
        tmp = AIR;
    }

    if(tmp != state)
    {
        state_tmp = state;
        state     = tmp;
        ROS_INFO_STREAM("" << state2str(state_tmp) << " => " << state2str(state) );
    }

}

/*
void State_machine::update_contact(){


    if(peg_sensor_listener.Y.n_elem > 1){

        if(peg_sensor_listener.Y(0) == 1){
            state.set_contact(true);
        }else{
            state.set_contact(false);
        }
    }

}*/


}
