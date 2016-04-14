#ifndef PEG_HOLE_POLICY_H_
#define PEG_HOLE_POLICY_H_

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

#include "policies/find_table.h"
#include "policies/find_socket.h"

#include "velocity_controller.h"

#include "robot_planners/velocity_reguliser.h"
#include "robot_planners/gmmPlanner.h"
#include "robot_planners/velocity_reguliser.h"


#include "robot_motion_generation/angular_velocity.h"
#include "exploration_planner/belief_gmm_planner.h"

#include "optitrack_rviz/listener.h"
#include "visualise/vis_vector.h"

#include "peg_hole_policy/cdd_filterConfig.h"
#include "peg_hole_policy/String_cmd.h"
#include "peg_hole_policy/policies/search_policy.h"

#include "netft_rdt_driver/ft_listener.h"
#include "netft_rdt_driver/String_cmd.h"

#include <optitrack_rviz/listener.h>
#include <robot_motion_generation/CDDynamics.h>

#include <lwr_ros_interface/ros_ee_j.h>
#include <lwr_ros_action/base_action.h>
#include "lwr_ros_interface/switch_controller.h"

#include <boost/scoped_ptr.hpp>
#include <memory>


namespace ph_policy{


enum class policy{
    NONE,
    SEARCH_POLICY
};

enum class ctrl_types{
    CARTESIAN,
    JOINT_POSITION
};

enum class CART_VEL_TYPE{
    OPEN_LOOP,
    PASSIVE_DS
};


class Peg_hole_policy  : public ros_controller_interface::Ros_ee_j, public ac::Base_action {


public:

    Peg_hole_policy(ros::NodeHandle& nh,
                    const std::string &fixed_frame,
                    belief::Gmm_planner& gmm_planner,
                    Peg_world_wrapper &peg_world_wrapper);

   virtual bool update();

   virtual bool stop();

private:

    void set_command(const Eigen::Vector3d& linear_vel_cmd,
                     const Eigen::Vector3d& angular_vel_cmd,
                     const Eigen::Quaterniond& orientation_cmd);

private:

    void cdd_callback(peg_hole_policy::cdd_filterConfig& config, uint32_t level);

    bool cmd_callback(peg_hole_policy::String_cmd::Request& req, peg_hole_policy::String_cmd::Response& res);

    void x_des_callback(const geometry_msgs::Pose &pos);

    void ft_classifier_callback(const std_msgs::Float32MultiArray &msg);

    bool net_ft_reset_bias();

    void disconnect();

private:

    tf::Vector3             current_dx;
    tf::Vector3             des_origin_;
    tf::Quaternion          des_orient_WF;
    tf::Vector3             current_origin_WF, current_origin_tmp;
    tf::Quaternion          current_orient_WF;

    // desired position and orientation given from the open loop controller
    tf::Vector3             x_des_q_;
    tf::Quaternion          q_des_q_;

    geometry_msgs::Pose     des_ee_pos_msg;
    geometry_msgs::Twist    des_ee_vel_msg;   /// desired end-effector velocities


    opti_rviz::Listener     ee_peg_listener;
    ros::Subscriber         x_des_subscriber;


    double              control_rate;
    bool                initial_config;
    std::size_t         tf_count;



    std::string         world_frame;

    ros::ServiceServer  server_srv;

    Eigen::Vector3d     linear_vel_cmd_;
    Eigen::Vector3d     angular_vel_cmd_;
    Eigen::Quaterniond  orientation_cmd_;

    /// SENSORS

    netft::Ft_listener              ft_listener_;

    ros::Subscriber                 ft_classifier_sub_;
    arma::colvec3                   Y_c;

    geometry_msgs::Wrench           wrench_;
    ros::ServiceClient              net_ft_sc_;
    netft_rdt_driver::String_cmd    net_ft_msg;

    /// POLICIES

    policy                          current_policy;
    ctrl_types                      ctrl_type;

    State_machine                   state_machine;
    Search_policy                   search_policy;

    /// Visualise direction

    opti_rviz::Vis_vectors              vis_vector;
    std::vector<opti_rviz::Arrow>       arrows;

    ros_controller_interface::Switch_controller switch_controller;

};


}



#endif
