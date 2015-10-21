#ifndef DEFAULT_TOPICS_H_
#define DEFAULT_TOPICS_H_

#include <string>
/*
#define EE_STATE_POSE_TOPIC "/joint_to_cart/est_ee_pose"
#define EE_STATE_FT_TOPIC "/joint_to_cart/est_ee_ft"
#define EE_CMD_POSE_TOPIC   "/cart_to_joint/des_ee_pose"
#define EE_CMD_FT_TOPIC   "/cart_to_joint/des_ee_ft"
#define BASE_LINK			"/base_link"
*/
class topics {

public:

    static const std::string EE_STATE_POSE_TOPIC;
    static const std::string EE_STATE_FT_TOPIC;
    static const std::string EE_CMD_POSE_TOPIC;
    static const std::string EE_CMD_FT_TOPIC ;
    static const std::string BASE_LINK;
};

const std::string topics::EE_STATE_POSE_TOPIC   = "/joint_to_cart/est_ee_pose";
const std::string topics::EE_STATE_FT_TOPIC     = "/joint_to_cart/est_ee_ft";
const std::string topics::EE_CMD_POSE_TOPIC     = "/cart_to_joint/des_ee_pose";
const std::string topics::EE_CMD_FT_TOPIC       = "/cart_to_joint/des_ee_ft";
const std::string topics::BASE_LINK             = "/base_link";



//#define FORCE_SCALING           3.0
//#define MAX_ROLLING_FORCE       30
//#define FORCE_WAIT_TOL          5

#endif
