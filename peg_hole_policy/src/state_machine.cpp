#include "peg_hole_policy/state_machine.h"

namespace ph_policy{

State_machine::State_machine(ros::NodeHandle &nh,const std::string& y_topic):
peg_sensor_listener(nh,y_topic)
{


}

void State_machine::update_contact(){


    if(peg_sensor_listener.Y.n_elem > 1){

        if(peg_sensor_listener.Y(0) == 1){
            state.set_contact(true);
        }else{
            state.set_contact(false);
        }
    }



}

void State_machine::update_location(){



}


void State_machine::update(){

    update_contact();
    update_location();

}


}
