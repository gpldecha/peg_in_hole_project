#ifndef PEG_HOLE_POLICY_H_
#define PEG_HOLE_POLICY_H_

#include <ros/ros.h>
#include <kuka_action_server/action_server.h>
#include <kuka_action_server/base_ee_action.h>
#include <kuka_common_action_server/action_initialiser.h>
#include <std_msgs/Float64MultiArray.h>


#include "state_machine.h"
#include "policies/find_table.h"
#include "policies/find_socket.h"
#include "policies/insert_peg.h"

#include "robot_planners/velocity_reguliser.h"
#include "peg_hole_policy/String_cmd.h"
#include "robot_motion_generation/angular_velocity.h"

namespace ph_policy{

typedef enum policies{
    NONE,
    FIND_TABLE,
    FIND_SOCKET,
    FIND_HOLE
}policies;


class Peg_hole_policy : public asrv::Base_ee_action, public asrv::Base_action_server {

public:

    Peg_hole_policy(ros::NodeHandle& nh, const std::string &path_sensor_model, const std::string &fixed_frame);

    virtual bool execute_CB(asrv::alib_server& as_,asrv::alib_feedback& feedback,const asrv::cptrGoal& goal);

private:

    bool cmd_callback(peg_hole_policy::String_cmd::Request& req, peg_hole_policy::String_cmd::Response& res);

private:

    tf::Pose                des_ee_pose;      /// desired end-effector position
    geometry_msgs::Twist    des_ee_vel_msg;   /// desired end-effector velocities


    double              control_rate;
    double              reachingThreshold;
    double              orientationThreshold;

    bool                initial_config;
    std::size_t         tf_count;

    policies            current_policy;

    State_machine       state_machine;

    std::string         world_frame;

    ros::ServiceServer  server_srv;



    /// POLICIES

    Find_table              find_table;
    Find_socket             find_socket;
    Insert_peg              insert_peg;
    Velocity_reguliser      velocity_reguliser;



};


}



#endif
