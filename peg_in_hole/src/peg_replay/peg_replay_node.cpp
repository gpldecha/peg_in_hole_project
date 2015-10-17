#include <ros/ros.h>

#include <optitrack_rviz/input.h>
#include <peg_in_hole/peg_replay.h>
#include <armadillo>
#include <boost/lexical_cast.hpp>


int main(int argc, char** argv){


    std::map<std::string,std::string> input;
    input["-traj_path"]        = "/home/guillaume/MatlabWorkSpace/peg_in_hole/TextData/Albert/";
    input["-rate"]             = "100";
    input["-broadcast_plug"]   = "link_cylinder";
    input["-broadcast_socket"] = "link_socket";

   if(!opti_rviz::Input::process_input(argc,argv,input)){
        ROS_ERROR("failed to load input");
        return 0;
    }
    opti_rviz::Input::print_input_options(input);

    ros::init(argc, argv, "peg_in_hole_replay");
    ros::NodeHandle node;
    ros::Rate rate(boost::lexical_cast<float>(input["-rate"]));


    Peg_replay peg_replay(node,input["-broadcast_plug"],input["-broadcast_socket"]);
    peg_replay.set_traj_dir_path(input["-traj_path"]);


    while(node.ok()){


        peg_replay.update();

        ros::spinOnce();
        rate.sleep();
    }






    return 0;
}
