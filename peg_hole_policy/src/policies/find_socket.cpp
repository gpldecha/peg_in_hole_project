#include "peg_hole_policy/policies/find_socket.h"
#include <limits>

namespace ph_policy{

Find_socket::Find_socket(ros::NodeHandle& nh,const std::string& path_sensor_model,const std::string& fixed_frame):
    Base_find(nh,path_sensor_model,fixed_frame)
{
    ptr_socket_wbox = &(peg_world_wrapper.get_wrapped_objects().get_wbox("wbox_socket"));



}

void Find_socket::get_linear_velocity(tf::Vector3& velocity,const tf::Vector3& peg_origin){

    peg_position(0) = peg_origin.getX();
    peg_position(1) = peg_origin.getY();
    peg_position(2) = peg_origin.getZ();

    ptr_socket_wbox->distance_to_surfaces(peg_position);
    direction       = (ptr_socket_wbox->get_surface_projection() - peg_position);
    distance_target =  arma::norm(direction,2);
    direction       =  direction/(distance_target + std::numeric_limits<double>::min());
    speed           =  velocity_reguliser.bel_shape_curve(distance_target,beta_vel_reg);

    ROS_INFO("speed: %f m/s",speed);
    direction = speed * direction;
//    direction.print("direction");

    velocity.setX(direction(0));
    velocity.setY(direction(1));
    velocity.setZ(direction(2));

}




}
