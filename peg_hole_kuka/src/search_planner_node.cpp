#include "search_planner/search_planner_node.h"
#include <string>

namespace spl{

Search_planner_node::Search_planner_node(ros::NodeHandle& nh_)
{

    sub_    = nh_.subscribe<geometry_msgs::PoseStamped>(EE_STATE_POSE_TOPIC, 1, &Search_planner_node::eeStateCallback,this);
    pub_    = nh_.advertise<geometry_msgs::PoseStamped>(EE_CMD_POSE_TOPIC, 1);
    pub_ft_ = nh_.advertise<geometry_msgs::WrenchStamped>(EE_CMD_FT_TOPIC, 1);



}

void Search_planner_node::eeStateCallback(const geometry_msgs::PoseStampedConstPtr& msg){
    const geometry_msgs::PoseStamped* data = msg.get();
    ee_pose.setOrigin(tf::Vector3(data->pose.position.x,data->pose.position.y,data->pose.position.z));
    ee_pose.setRotation(tf::Quaternion(data->pose.orientation.x,data->pose.orientation.y,data->pose.orientation.z,data->pose.orientation.w));

    ROS_INFO("Search_planner_node::eeStateCallback");
    isOkay = true;
}

void Search_planner_node::sendPose(const tf::Pose& pose_) {
    geometry_msgs::PoseStamped msg;
    msg.pose.position.x = pose_.getOrigin().x();
    msg.pose.position.y = pose_.getOrigin().y();
    msg.pose.position.z = pose_.getOrigin().z();

    msg.pose.orientation.x = pose_.getRotation().x();
    msg.pose.orientation.y = pose_.getRotation().y();
    msg.pose.orientation.z = pose_.getRotation().z();
    msg.pose.orientation.w = pose_.getRotation().w();
    pub_.publish(msg);
}


}


