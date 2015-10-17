// ROS

#include<ros/ros.h>

// catkin package

#include "optitrack_rviz/input.h"
#include "optitrack_rviz/listener.h"
#include "optitrack_rviz/broadcaster.h"
#include "optitrack_rviz/filter.h"
#include "optitrack_rviz/load.h"

// STL

#include <map>
#include <string>
#include <iostream>

// Boost

#include <boost/lexical_cast.hpp>




int main(int argc,char** argv)
{

    std::map<std::string,std::string> input;

    input["-fixed_frame"]               = "world";
    input["-target_frame_listener"]     = "";
    input["-target_frame_broadcaster"]  = "";
    input["-origin"]                    = "0 0 0";
    input["-orientation"]               = "0 0 0 1";
    input["-rate"]                      = "100";
    input["-save"]                      = "/home/guillaume/";
    input["-load"]                      = "False";

    std::array<float,3> origin,offset;
    std::array<float,4> orientation;
    offset = {{0,0,0}};


    if(!opti_rviz::Input::process_input(argc,argv,input,origin,orientation,offset)){
        return -1;
    }

    int Hz = boost::lexical_cast<int>(input["-rate"]);



    opti_rviz::Input::print_input_options(input);

    bool bLoad = false;
    tf::Vector3     T;
    tf::Quaternion  q;

    if(input["-load"] == "True"){
        std::cout<< "== load == " << std::endl;
        bLoad=true;
        opti_rviz::Load::load(T,q,input["-save"]);
    }

    ros::init(argc, argv,"~",ros::init_options::AnonymousName);
    ros::NodeHandle node;

    tf::Vector3     origin_plug;
    tf::Matrix3x3   orientation_plug,R;

    opti_rviz::Listener            listener(input["-fixed_frame"],input["-target_frame_listener"]);
    opti_rviz::Broadcaster         broadcaster(input["-fixed_frame"],input["-target_frame_broadcaster"]);

    opti_rviz::Quaternion_filter   q_filter(0.1);
    opti_rviz::Kalman              kalman(1.0/(float)Hz,1,100);
    opti_rviz::Jumps               jump(0.1,0.1,false);


    opti_rviz::Attractors          attractor_filter;
    opti_rviz::Attractors::attractor attrac;
    attrac.stiff = 100;
    attrac.z_axis = tf::Vector3(0,0,1);
    attractor_filter.push_back(attrac);

    q.setEuler(0,0,0);


    listener.update_opti2rviz(origin_plug,orientation_plug);
    ros::spinOnce();
    kalman.init(origin_plug);


    ros::Rate rate(Hz);
    while(node.ok()){

        listener.update_opti2rviz(origin_plug,orientation_plug);
        orientation_plug.getRotation(q);

       // jump.update(origin_plug,q);


        kalman.update(origin_plug);
        q_filter.update(q);


        if(bLoad){
            origin_plug = origin_plug - T;
        }

        orientation_plug.setRotation(q);
        broadcaster.update(origin_plug,orientation_plug);


        ros::spinOnce();
        rate.sleep();
    }



    return 0;
}
