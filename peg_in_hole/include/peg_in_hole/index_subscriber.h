#ifndef INDEX_SUBSCRIBER_H_
#define INDEX_SUBSCRIBER_H_

#include <ros/ros.h>
#include <std_msgs/UInt32.h>

class Index_subscriber{

public:

    Index_subscriber(ros::NodeHandle& node,const std::string& topic_name);

private:

    void callback(const std_msgs::UInt32::ConstPtr& msg);

public:

    std::size_t index;

private:

    ros::Subscriber index_subscriber;


};


#endif
