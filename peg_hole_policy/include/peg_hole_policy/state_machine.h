#ifndef PEG_STATE_MACHINE_H_
#define PEG_STATE_MACHINE_H_

#include <ros/ros.h>
#include "peg_sensor/listener.h"
#include <queue>
#include <Eigen/Eigen>
#include "geometry_msgs/WrenchStamped.h"

namespace ph_policy{

typedef enum State{
    AIR,
    CONTACT,
    EDGE
}State;

inline std::string state2str(State state)
{
    switch(state)
    {
    case AIR:
    {
        return "AIR";
    }
    case CONTACT:
    {
        return "CONTACT";
    }
    case EDGE:
    {
        return "EDGE";
    }
    }
}


/*
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
*/


class State_machine{

public:

    State_machine(ros::NodeHandle& nh, const std::string &y_topic);

    void update(Eigen::Vector3d& Y_c,geometry_msgs::Wrench& wrench);

private:


    State state, state_tmp,tmp;


private:

    psm::Peg_sensor_listener peg_sensor_listener;
};

}

#endif
