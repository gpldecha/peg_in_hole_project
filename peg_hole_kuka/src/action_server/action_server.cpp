#include "action_server/action_server.h"

namespace asrv {

Action_server::Action_server(ros::NodeHandle& nh,std::string name):
    as_(nh, name, boost::bind(&Action_server::executeCB, this, _1), false),
    action_name_(name)
{
    as_.start();
}

void Action_server::push_back(fexecuteCB& function,std::string action_name){
    actions[action_name] = &function;
}

void Action_server::executeCB(const cptrGoal& goal){

    std::string desired_action = goal->action_type;
    ROS_INFO_STREAM( "Desired Action is " << desired_action);

   /* (*ptr_isOkay) = false;
    ros::Rate r(10);
    ROS_INFO("Waiting for EE pose/ft topic...");
    while(ros::ok() && (!(*ptr_isOkay))) {
        r.sleep();
    }*/

    if(!ros::ok()) {
        result_.success = 0;
        ROS_INFO("%s: Failed", action_name_.c_str());
        as_.setAborted(result_);
        return;
    }

    // initialize action progress as null
    feedback_.progress = 0;

    ///////////////////////////////////////////////
    /////----- EXECUTE REQUESTED ACTION ------/////
    ///////////////////////////////////////////////

    std::string action_type = goal->action_type;
    actions_it              = actions.find(action_type);

    if(actions_it == actions.end()){
        ROS_ERROR_STREAM("Unidentified action name "<< action_type.c_str());
        result_.success = false;
        as_.setAborted(result_);
    }else{

        fexecuteCB& action_function = *(actions_it->second);
        bool success    = action_function(as_,feedback_,goal);
        result_.success = success;
        if(success)
        {
            ROS_INFO("%s: Succeeded", action_name_.c_str());
            as_.setSucceeded(result_);
        } else {
            ROS_INFO("%s: Failed", action_name_.c_str());
            as_.setAborted(result_);
        }
    }
}





}
