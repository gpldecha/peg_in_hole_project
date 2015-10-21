#include "action_server/action_server.h"
#include "motion_actions/pour_action.h"

#include <functional>


int main(int argc, char** argv) {

    ros::init(argc, argv, "plan2ctrl");
    ROS_INFO("Initializing Server");
    ros::NodeHandle nh;

    asrv::Action_server action_server(nh,ros::this_node::getName());

    Pour_action pour_action(nh);
    pour_action.initialize();

    asrv::fexecuteCB learned_model_function = std::bind(&Pour_action::executeCB,
                                                        &pour_action,
                                                        std::placeholders::_1,
                                                        std::placeholders::_2,
                                                        std::placeholders::_3);

    action_server.push_back(learned_model_function,"LEARNED_MODEL");

    ros::spin();

    return 0;
}
