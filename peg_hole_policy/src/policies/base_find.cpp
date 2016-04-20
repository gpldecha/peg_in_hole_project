#include "peg_hole_policy/policies/base_find.h"

namespace ph_policy{

Base_find::Base_find(ros::NodeHandle& nh,const std::string& path_sensor_model,const std::string& fixed_frame,const std::string& peg_frame):
     peg_world_wrapper(nh,false,"peg_sensor_classifier",path_sensor_model,fixed_frame,peg_frame)
{

    velocity_reguliser.set_max_speed_ms(0.025);
    velocity_reguliser.set_min_speed_ms(0.001);
    beta_vel_reg = 10;
}

}
