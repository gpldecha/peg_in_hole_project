#include "peg_hole_policy/policies/insert_peg.h"

namespace ph_policy{


Insert_peg::Insert_peg(ros::NodeHandle& nh,const std::string& path_sensor_model,const std::string& fixed_frame)
    :Base_find(nh,path_sensor_model,fixed_frame)
{

    tf::StampedTransform transform;
    opti_rviz::Listener::get_tf_once(fixed_frame,"link_socket",transform);
    Rt.setRotation(transform.getRotation());
    Rt = Rt.transpose();
    T = transform.getOrigin();

    // load gmr model
    std::string path_gmm_parameters = "/home/guillaume/MatlabWorkSpace/peg_in_hole_RL/PolicyModelSaved/PolicyModel_txt/gmm_xsocket";
    std::vector<std::size_t>  in  = {{0,1,2}};
    std::vector<std::size_t>  out = {{3,4,5}};
    gmr_policy = planners::GMR_EE_Planner(path_gmm_parameters,in,out);

    std::cout<< "after loading gmr_model" << std::endl;
    input.resize(3);

    std::cout<< "=== Test GMR Policy === " << std::endl;
    input(0) = 0.06;
    input(1) = 0.04;
    input(2) = 0;

    gmr_policy.gmr(input);
    gmr_policy.get_ee_linear_velocity(vel_direction);

}

void Insert_peg::get_linear_velocity(tf::Vector3& velocity,const tf::Vector3& peg_origin){
    pos_fr_socket = Rt * (peg_origin - T);

    input(0) = pos_fr_socket.getX();
    input(1) = pos_fr_socket.getY();
    input(2) = pos_fr_socket.getZ();

    gmr_policy.gmr(input);
    vel_direction.zeros();
    gmr_policy.get_ee_linear_velocity(vel_direction);
    vel_direction = vel_direction / (arma::norm(vel_direction,2) + std::numeric_limits<double>::min() );

    direction(0) = vel_direction(0);
    direction(1) = vel_direction(1);
    direction(2) = vel_direction(2);


    distance_target =  T.distance(peg_origin);
   // direction       =  direction/(distance_target + std::numeric_limits<double>::min());
    speed           =  velocity_reguliser.bel_shape_curve(distance_target,beta_vel_reg);

    if(std::isnan(speed)){
        ROS_ERROR("speed is NAN");
    }


   // ROS_INFO("speed: %f m/s  dist: %f",speed,distance_target);
    direction = speed * direction;

    velocity.setX(direction(0));
    velocity.setY(direction(1));
    velocity.setZ(direction(2));

}


void Insert_peg::get_linear_velocity(tf::Vector3 &velocity, const tf::Vector3 &peg_origin,tf::Vector3& des_origin,tf::Quaternion& des_orient){
    get_linear_velocity(velocity,peg_origin);
    des_origin = T;
    des_orient.setRPY(0,0,0);
}


}
