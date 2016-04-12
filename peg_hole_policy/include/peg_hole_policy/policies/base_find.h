#ifndef PEG_BASE_FIND_H_
#define PEG_BASE_FIND_H_

#include <peg_sensor/peg_world_wrapper/peg_world_wrapper.h>
#include <tf/LinearMath/Vector3.h>

#include "robot_planners/velocity_reguliser.h"


namespace ph_policy{

class Base_find{

public:

    Base_find(ros::NodeHandle& nh, const std::string& path_sensor_model, const std::string& fixed_frame, const std::string &peg_frame);

    virtual void get_linear_velocity(tf::Vector3& velocity,const tf::Vector3& peg_origin) = 0;

public:

    Velocity_reguliser      velocity_reguliser;
    double                  beta_vel_reg;


protected:

       Peg_world_wrapper       peg_world_wrapper;
       geo::fCVec3             peg_position;
       geo::fCVec3             direction;
       double                  speed;
       double                  distance_target;


};


}


#endif
