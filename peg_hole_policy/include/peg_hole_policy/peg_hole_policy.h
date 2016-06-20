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
#include <optitrack_rviz/publisher.h>

#include "peg_hole_policy/cdd_filterConfig.h"
#include "peg_hole_policy/String_cmd.h"
#include "peg_hole_policy/policies/search_policy.h"
#include "peg_hole_policy/policies/specialised_policy.h"
#include "peg_hole_policy/policies/demo_policies.h"

#include "record_ros/String_cmd.h"

#include "netft_rdt_driver/ft_listener.h"
#include "netft_rdt_driver/String_cmd.h"

#include <optitrack_rviz/listener.h>

#include <lwr_ros_action/base_action.h>


#include <lwr_ros_interface/ros_ee_j.h>
#include "lwr_ros_interface/switch_controller.h"
#include "lwr_ros_interface/ros_passive_ds.h"

#include <boost/scoped_ptr.hpp>
#include <memory>


namespace ph_policy{

enum class policy{
    NONE,
    SEARCH_POLICY
};

enum class ctrl_types{
    CARTESIAN,
    PASSIVE_DS,
    JOINT_POSITION
};

class Peg_hole_policy  : public ros_controller_interface::Ros_ee_j, public ac::Base_action {


public:

    Peg_hole_policy(ros::NodeHandle& nh,
                    const std::string& fixed_frame,
                    const std::string& ft_topic,
                    const std::string& ft_classifier_topic,
                    const std::string& belief_state_topic,
                    const std::string& record_topic_name,
                    Peg_sensor_model&  peg_sensor_model,
                    wobj::WrapObject& wrapped_objects,
                    SOCKET_TYPE socket_type);

   virtual bool update();

   virtual bool stop();

   void print_debug();

private:

    void set_command(const Eigen::Vector3d& linear_vel_cmd,
                     const Eigen::Vector3d& angular_vel_cmd,
                     const Eigen::Quaterniond &orientation_cmd);

private:

    bool cmd_callback(peg_hole_policy::String_cmd::Request& req, peg_hole_policy::String_cmd::Response& res);

    void x_des_callback(const geometry_msgs::Pose &pos);

    void sensor_classifier_callback(const std_msgs::Float64MultiArray &msg);

    void belief_state_callback(const std_msgs::Float64MultiArrayConstPtr &msg);

    void check_record(bool start);

    void openloopx_callback(const geometry_msgs::PoseStampedConstPtr& msg);

    void reset_belief(std::string &res);

private:

    void reset();

    void smooth_velocity();

    bool net_ft_reset_bias();

    void disconnect();

    void rviz_velocity();

    void set_angular_velocity();

private:

    ros_controller_interface::Ros_passive_ds ros_passive_ds;

private:

    tf::Vector3             current_dx;
    tf::Vector3             des_origin_;
    tf::Quaternion          des_orient_WF;
    tf::Vector3             current_origin_WF, current_origin_tmp;
    tf::Quaternion          current_orient_WF;
    tf::Quaternion          target_orient_WF;
    tf::Quaternion          qdiff;
    tf::Vector3             velocity, velocity_tmp;

    arma::colvec            belief_state_WF;
    arma::colvec            belief_state_SF;
    arma::colvec3           socket_pos_WF;
    arma::colvec3           current_origin_SF;
    arma::colvec3           arma_velocity;

    opti_rviz::Publisher    pub_ee_pos_SF;
    opti_rviz::Publisher    pub_belief_SF;

    arma::colvec3           force;
    arma::colvec3           open_loop_x_origin_arma_WF;
    tf::Vector3             open_loop_x_origin_tf;
    tf::Quaternion          open_loop_x_orient_tf;


    Search_policy::POLICY search_policy_type;

    // desired position and orientation given from the open loop controller
    tf::Vector3             x_des_q_;
    tf::Quaternion          q_des_q_;

    geometry_msgs::Pose     des_ee_pos_msg;
    geometry_msgs::Twist    des_ee_vel_msg;   /// desired end-effector velocities


    opti_rviz::Listener     ee_peg_listener;

    Peg_sensor_model&       peg_sensor_model;

    ros::Subscriber                                 sensor_classifier_sub_;
    ros::Subscriber                                 openloopx_sub;
    ros::Subscriber                                 x_des_subscriber;
    ros::Subscriber                                 belief_info_sub;
    ros::Publisher                                  belief_mode_reset_pub_;



    double                                          control_rate;
    bool                                            initial_config;
    bool                                            bFirst;
    std::size_t                                     tf_count;

    arma::colvec3                                   T;
    arma::mat33                                     Rt;
    arma::colvec3                                   pos_tmp,arma_current_origin_WF;

    bool                                            bRecord;
    ros::ServiceClient                              record_client;



    std::string                                     world_frame;

    ros::ServiceServer                              server_srv;

    Eigen::Vector3d                                 linear_vel_cmd_;
    Eigen::Vector3d                                 angular_vel_cmd_;
    Eigen::Quaterniond                              orientation_cmd_;

    /// SENSORS

    netft::Ft_listener                              ft_listener_;
    arma::colvec                                    Y_c;

    geometry_msgs::Wrench                           wrench_;
    ros::ServiceClient                              net_ft_sc_;
    netft_rdt_driver::String_cmd                    net_ft_msg;

    /// POLICIES

    boost::shared_ptr<ph_policy::Demo_policies>     demo_policy;
    boost::shared_ptr<ph_policy::Specialised>       specialised_policy;

    boost::shared_ptr<ph_policy::GMM>               gmm_policy;
    boost::scoped_ptr<ph_policy::Search_policy>     search_policy;
    ph_policy::Get_back_on                          get_back_on;


    policy                                          current_policy;
    ctrl_types                                      ctrl_type;

    State_machine                                   state_machine;

    /// Visualise direction

    opti_rviz::Vis_vectors                          vis_vector;
    std::vector<opti_rviz::Arrow>                   arrows;

    ros_controller_interface::Switch_controller     switch_controller;

    SOCKET_TYPE                                     socket_type;

};


}



#endif
