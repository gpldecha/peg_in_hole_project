#ifndef SEARCH_PLANNER_H_
#define SEARCH_PLANNER_H_

#include <string>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>

//-- Custom ActionLib Stuff --//
#include "actionlib/server/simple_action_server.h"
#include <lasa_action_planners/PLAN2CTRLAction.h>


namespace spl{

// this should be the actionlib library

class Search_planner{

public:


public:

    Search_planner(ros::NodeHandle& nh_,std::string name);

    ~Search_planner();

};

}

#endif
