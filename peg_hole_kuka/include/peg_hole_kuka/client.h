#ifndef KUKA_PEG_ACTION_CLIENT_H_
#define KUKA_PEG_ACTION_CLIENT_H_


#include "kuka_action_client/kuka_action_client.h"
#include <geometry_msgs/Transform.h>
#include <Eigen/Core>
#include <optitrack_rviz/listener.h>

class PEG_action_client{

public:

    PEG_action_client();


    void initialise();

    void init_cart();

    void init_simple_bel_planner();

    void get_link_socket_goal(tf::Transform& transform);

public:

    std::map<std::string,ac::Goal> goals;


};


#endif
