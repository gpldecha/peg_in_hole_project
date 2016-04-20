#include "peg_hole_policy/stack_planner/planning_stack.h"

namespace ph_policy{

Planning_stack::Planning_stack(){
    reset();
}

void Planning_stack::reset(){
    stack = {{FIND_TABLE,GO_TO_EDGE,GO_TO_SOCKET,FIND_SOCKET_HOLE,INSERT}};
    index = 0;
    current_action = stack[index];
    bFrist         = true;
}

void Planning_stack::set_action(actions action){
    for(std::size_t i = 0; i < stack.size();i++){
        if(stack[i] == action){
            index = i;
            std::cout<< "found action: " << actions2str(action) << std::endl;
            return;
        }
    }

    std::cout<< "did not find action: " << actions2str(action) << std::endl;
}

void Planning_stack::update(const std::vector<STATES>& states){

    current_action = stack[index];

    if(current_action == FIND_TABLE){
        if(has_state(STATES::TABLE,states)){
            next_action();
        }
    }else if(current_action == GO_TO_EDGE){
        if(has_state(STATES::CLOSE_EDGE,states)){
            next_action();
        }
    }else if(current_action == GO_TO_SOCKET){
        if(has_state(STATES::SOCKET,states) || has_state(STATES::MODE_CLOSE_HOLE,states)){
            next_action();
        }
    }else if(current_action == FIND_SOCKET_HOLE){
        if(has_state(STATES::SOCKET,states)){
            //   next_action();
        }
    }else if(current_action == INSERT){

    }

    if(has_state(STATES::OFF_TABLE,states)){
        current_action = GET_BACK_ONTO_TABLE;
    }
}

bool Planning_stack::has_state(const STATES state,const std::vector<STATES>& states){
    return std::find(states.begin(),states.end(),state) != states.end();
}

const void Planning_stack::print(double seconds) const{
    if(seconds==0){
        ROS_INFO_STREAM("action: " << actions2str(get_action()));
    }else{
        ROS_INFO_STREAM_THROTTLE(seconds,"action: " << actions2str(get_action()));
    }
}

actions Planning_stack::get_action() const{
    return current_action;
}

void Planning_stack::next_action(){
    index++;
    if(index >= stack.size()){
        ROS_WARN_THROTTLE(1.0,"planning stack reached end!");
        index=stack.size()-1;
    }
    ROS_INFO_STREAM("next action : [" << actions2str(get_action()) << "]");
}

}
