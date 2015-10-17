#include <peg_in_hole/index_subscriber.h>


Index_subscriber::Index_subscriber(ros::NodeHandle& node,const std::string& topic_name){

    index_subscriber = node.subscribe(topic_name, 100, &Index_subscriber::callback,this);
    index            = 0;
}

void Index_subscriber::callback(const std_msgs::UInt32::ConstPtr &msg){
    index = msg->data;
}
