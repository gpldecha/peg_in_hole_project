#include "motion_actions/base_action.h"

#define EE_STATE_POSE_TOPIC "/joint_to_cart/est_ee_pose"
#define EE_STATE_FT_TOPIC "/joint_to_cart/est_ee_ft"
#define EE_CMD_POSE_TOPIC   "/cart_to_joint/des_ee_pose"
#define EE_CMD_FT_TOPIC   "/cart_to_joint/des_ee_ft"
#define BASE_LINK			"/base_link"


Base_action::Base_action(ros::NodeHandle&   nh,
                         const std::string& ee_state_pos_topic,
                         const std::string& ee_cmd_pos_topic,
                         const std::string& ee_cmd_ft_topic){

    ee_ft.resize(6);
    // ROS TOPICS for controllers
    sub_        = nh.subscribe<geometry_msgs::PoseStamped>(ee_state_pos_topic, 1, &Base_action::eeStateCallback,this);
    pub_        = nh.advertise<geometry_msgs::PoseStamped>(ee_cmd_pos_topic, 1);
    pub_ft_     = nh.advertise<geometry_msgs::WrenchStamped>(ee_cmd_ft_topic, 1);

}

void Base_action::eeStateCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
    const geometry_msgs::PoseStamped* data = msg.get();
    ee_pose.setOrigin(tf::Vector3(data->pose.position.x,data->pose.position.y,data->pose.position.z));
    ee_pose.setRotation(tf::Quaternion(data->pose.orientation.x,data->pose.orientation.y,data->pose.orientation.z,data->pose.orientation.w));
    isOkay = true;
}


void Base_action::toPose(const MathLib::Matrix4& mat4, tf::Pose& pose) {
    MathLib::Matrix3 m1 = mat4.GetOrientation();
    MathLib::Vector3 v1 = m1.GetRotationAxis();
    tf::Vector3 ax(v1(0), v1(1), v1(2));
    pose.setRotation(tf::Quaternion(ax, m1.GetRotationAngle()));
    v1.Set(mat4.GetTranslation());
    pose.setOrigin(tf::Vector3(v1(0),v1(1),v1(2)));
}

MathLib::Matrix4 Base_action::toMatrix4(const tf::Pose& pose) {
    MathLib::Matrix4 mat;
    mat.Identity();
    tf::Matrix3x3 mat33(pose.getRotation());

    mat.SetTranslation(MathLib::Vector3(pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z()));
    mat.SetOrientation(MathLib::Matrix3(mat33[0][0], mat33[0][1], mat33[0][2],
            mat33[1][0], mat33[1][1], mat33[1][2],
            mat33[2][0], mat33[2][1], mat33[2][2]));
    return mat;
}


// The famous sendPose!
void Base_action::sendPose(const tf::Pose& pose_) {
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
