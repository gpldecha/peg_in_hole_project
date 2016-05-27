#include "peg_hole_policy/policies/find_table.h"

namespace ph_policy{


Find_table::Find_table(Peg_world_wrapper&       peg_world_wrapper):
    Base_find(peg_world_wrapper)
{

    peg_world_wrapper.get_wrapped_objects().print_info(wobj::NAMES);
    velocity_reguliser.set_max_speed_ms(0.1);

    ptr_table_wbox = &(peg_world_wrapper.get_wrapped_objects().get_wbox("link_wall"));
    bFirst = true;
}

void Find_table::get_linear_velocity(tf::Vector3 &velocity, const tf::Vector3& peg_origin){

    peg_position(0) = peg_origin.getX();
    peg_position(1) = peg_origin.getY();
    peg_position(2) = peg_origin.getZ();
    velocity_reguliser.set_max_speed_ms(0.1);


    if(bFirst){
        ptr_table_wbox->distance_to_surfaces(peg_position);
        surf_proj = ptr_table_wbox->get_surface_projection();
        bFirst=false;
    }

    direction       = (surf_proj - peg_position);
   // ROS_INFO_STREAM_THROTTLE(1.0,"surf_proj:       " << surf_proj(0) << " " << surf_proj(1) << " " << surf_proj(2));
   // ROS_INFO_STREAM_THROTTLE(1.0,"distance_target: " << distance_target);

    distance_target =  arma::norm(direction,2);
    direction       =  direction/(distance_target + std::numeric_limits<double>::min());
    speed           =  velocity_reguliser.bel_shape_curve(distance_target,beta_vel_reg);


   // ROS_INFO_STREAM_THROTTLE(1.0,"speed: " << speed << " m/s");
    direction = speed * direction;
    velocity.setX(direction(0));
    velocity.setY(direction(1));
    velocity.setZ(direction(2));
}

void Find_table::get_linear_velocity(tf::Vector3 &velocity, const tf::Vector3 &peg_origin,tf::Vector3& des_origin,tf::Quaternion& des_orient){

    get_linear_velocity(velocity,peg_origin);
    des_origin.setX(surf_proj(0));
    des_origin.setY(surf_proj(1));
    des_origin.setZ(surf_proj(2));
    des_orient.setRPY(0,0,0);
}




}
