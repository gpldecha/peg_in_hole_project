#ifndef PEG_HOLE_POLICY_H_
#define PEG_HOLE_POLICY_H_

#include <ros/ros.h>
#include <kuka_action_server/action_server.h>
#include <kuka_action_server/base_ee_j_action.h>
#include <kuka_common_action_server/action_initialiser.h>
#include <std_msgs/Float64MultiArray.h>


#include "state_machine.h"
#include "policies/find_table.h"
#include "policies/find_socket.h"
#include "policies/insert_peg.h"

#include "velocity_controller.h"

#include "robot_planners/velocity_reguliser.h"
#include "robot_planners/gmmPlanner.h"
#include "exploration_planner/belief_gmm_planner.h"
#include "robot_planners/velocity_reguliser.h"


#include "robot_motion_generation/angular_velocity.h"

#include "optitrack_rviz/listener.h"
#include <visualise/vis_vector.h>

#include "peg_hole_policy/cdd_filterConfig.h"

#include <boost/scoped_ptr.hpp>


#include "netft_rdt_driver/ft_listener.h"
#include "netft_rdt_driver/String_cmd.h"
#include "peg_hole_policy/String_cmd.h"


#include <memory>

namespace ph_policy{

typedef enum policies{
    NONE,
    FIND_TABLE,
    FIND_SOCKET,
    FIND_HOLE,
    SEARCH_POLICY
}policies;

typedef enum ctrl_types{
    CARTESIAN,
    JOINT_POSITION
} ctrl_types ;


class Peg_hole_policy : public asrv::Base_ee_j_action, public asrv::Base_action_server {


public:

    Peg_hole_policy(ros::NodeHandle& nh, const std::string &path_sensor_model, const std::string &fixed_frame);

    virtual bool execute_CB(asrv::alib_server& as_,asrv::alib_feedback& feedback,const asrv::cptrGoal& goal);

private:

    void set_command(const Eigen::Vector3d& linear_vel_cmd,
                     const Eigen::Vector3d& angular_vel_cmd,
                     const Eigen::Quaterniond& orientation_cmd);

private:

    void cdd_callback(peg_hole_policy::cdd_filterConfig& config, uint32_t level);

    bool cmd_callback(peg_hole_policy::String_cmd::Request& req, peg_hole_policy::String_cmd::Response& res);

    void ft_classifier_callback(const std_msgs::Float32MultiArray &msg);

    bool net_ft_reset_bias();

private:

    tf::Vector3             current_dx;
    tf::Vector3             des_origin_;
    tf::Quaternion          des_orient_WF;

    geometry_msgs::Pose     des_ee_pos_msg;
    geometry_msgs::Twist    des_ee_vel_msg;   /// desired end-effector velocities


    Velocity_reguliser      velocity_reguliser_;

    opti_rviz::Listener     ee_peg_listener;


    double              control_rate;
    double              reachingThreshold;
    double              orientationThreshold;

    bool                initial_config;
    std::size_t         tf_count;

    policies            current_policy;

    State_machine       state_machine;

    std::string         world_frame;

    ros::ServiceServer  server_srv;

    Eigen::Vector3d     linear_vel_cmd_;
    Eigen::Vector3d     angular_vel_cmd_;
    Eigen::Quaterniond  orientation_cmd_;

    Eigen::Vector3d     Y_c;

    netft::Ft_listener  ft_listener_;
    ros::Subscriber     ft_classifier_sub_;
    geometry_msgs::Wrench wrench_;
    ros::ServiceClient  net_ft_sc_;
    netft_rdt_driver::String_cmd net_ft_msg;


    /// POLICIES

    Find_table              find_table;
    Find_socket             find_socket;
    Insert_peg              insert_peg;
    Velocity_reguliser      velocity_reguliser;


    belief::Gmm_planner     bel_planner;
   /* planners::GMAPlanner    gma_planner;
    arma::colvec3           gma_velocity;
    arma::colvec            F;*/


    ctrl_types              ctrl_type;

    Velocity_controller     velocity_controller;

    boost::scoped_ptr<motion::CDDynamics> linear_cddynamics;
    boost::scoped_ptr<motion::CDDynamics> angular_cddynamics;

    ros::NodeHandle nd_cdd;
    boost::scoped_ptr< dynamic_reconfigure::Server< peg_hole_policy::cdd_filterConfig> >    dynamic_server_cdd_param;


    bool bGrav;
    std::vector<double> stiffness,stiffness_tmp;


    /// Visualise direction
    opti_rviz::Vis_vectors vis_vector;
    std::vector<opti_rviz::Arrow>       arrows;



};


}



#endif
