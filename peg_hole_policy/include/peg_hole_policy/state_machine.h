#ifndef PEG_STATE_MACHINE_H_
#define PEG_STATE_MACHINE_H_

#include <ros/ros.h>
#include "peg_sensor/listener.h"
#include <queue>

namespace ph_policy{

typedef enum Location{
    air,
    table,
    socket,
    hole
}Location;

class State{

    public:

    State(){

    }

    bool get_current_contact(){
        return b_contact.back();
    }

    bool get_past_contact(){
        return b_contact.front();
    }

    void set_contact(bool bcontact){
        if (b_contact.size() >= 2){

            if( b_contact.back() != b_contact.front() ){
                b_contact.push(bcontact);
                b_contact.pop();
            }

        }else{
            b_contact.push(bcontact);
        }

    }

    std::queue<bool> b_contact;

    Location    current_location;


};


class State_machine{


public:

    State_machine(ros::NodeHandle& nh, const std::string &y_topic);

    void update();

private:

    void update_contact();

    void update_location();

public:


    State state;


private:

    psm::Peg_sensor_listener peg_sensor_listener;
};

}

#endif
