#ifndef SEARCH_PLANNER_NODE_H_
#define SEARCH_PLANNER_NODE_H_

#include <ros/ros.h>

// TF

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>

// Message Types

#include <robohow_common_msgs/MotionPhase.h>
#include <robohow_common_msgs/MotionModel.h>
#include <robohow_common_msgs/GaussianMixtureModel.h>
#include <robohow_common_msgs/GaussianDistribution.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/WrenchStamped.h"

#define EE_STATE_POSE_TOPIC         "/joint_to_cart/est_ee_pose"
#define EE_STATE_FT_TOPIC           "/joint_to_cart/est_ee_ft"
#define EE_CMD_POSE_TOPIC           "/cart_to_joint/des_ee_pose"
#define EE_CMD_FT_TOPIC             "/cart_to_joint/des_ee_ft"
#define BASE_LINK                   "/base_link"
#define FORCE_SCALING               3.0
#define MAX_ROLLING_FORCE           30
#define FORCE_WAIT_TOL              5


namespace spl{

class Search_planner_node{

public:

    Search_planner_node(ros::NodeHandle &nh_);

    void eeStateCallback(const geometry_msgs::PoseStampedConstPtr& msg);

    void sendPose(const tf::Pose& pose_);

private:

    // for state callback
    tf::Pose                        ee_pose;
    tf::Pose                        curr_ee_pose;
    tf::Pose                        des_ee_pose;
    volatile bool                   isOkay, isFTOkay;


    // for subscribing / publising
    ros::Subscriber                 sub_, sub_ft_;
    ros::Publisher                  pub_, pub_ft_;
    geometry_msgs::PoseStamped      msg_pose;
    geometry_msgs::WrenchStamped    msg_ft;



};

}


#endif
