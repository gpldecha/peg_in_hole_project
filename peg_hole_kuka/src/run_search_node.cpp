#include "search_planner/search_planner.h"
#include "search_planner/search_planner_node.h"


int main(int argc,char** argv)
{

    std::cout << "HELLO" << std::endl;

    ros::init(argc, argv, "search_planner");
    ros::NodeHandle nh;
    spl::Search_planner_node search_planner_node(nh);

    ros::spin();

/*    ros::Rate rate(10.0);
    while(ros::ok())
    {

        ros::spinOnce();
        rate.sleep();
    }*/


    return 0;
}
