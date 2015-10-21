#ifndef BASE_ACTION_H_
#define BASE_ACTION_H_

#include <ros/ros.h>
//-- TF Stuff --//
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/WrenchStamped.h"

#include "MathLib/MathLib.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

class Base_action{

public:

    Base_action(ros::NodeHandle&   nh,
                const std::string& ee_state_pos_topic,
                const std::string& ee_cmd_pos_topic,
                const std::string& ee_cmd_ft_topic);

    void eeStateCallback(const geometry_msgs::PoseStampedConstPtr& msg);

    void sendPose(const tf::Pose& pose_);

    void toPose(const MathLib::Matrix4& mat4, tf::Pose& pose);

    MathLib::Matrix4 toMatrix4(const tf::Pose& pose);

public:

    ros::Subscriber                     sub_, sub_ft_;
    ros::Publisher                      pub_, pub_ft_;
    geometry_msgs::PoseStamped          msg_pose;
    geometry_msgs::WrenchStamped        msg_ft;


    tf::Pose                            ee_pose, curr_ee_pose, des_ee_pose;
    Eigen::VectorXd                     ee_ft;

    volatile bool                       isOkay, isFTOkay;

};

#endif
