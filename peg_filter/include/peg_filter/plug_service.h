#ifndef PLUG_SERVICE_H_
#define PLUG_SERVICE_H_

#include <tf/LinearMath/Matrix3x3.h>
#include <armadillo>
#include <ros/ros.h>
#include <peg_filter/pf_manager.h>
#include <optitrack_rviz/listener.h>

namespace plugfilter {

class Plug_service{

    enum{RESET,BETA,NOISE,START};

public:

    Plug_service(ros::NodeHandle& node,Plug_pf_manager& plug_pf_manager);

private:

    bool callback(particle_filter::String_cmd::Request& req,particle_filter::String_cmd::Response& res);

private:

    ros::ServiceServer                      service;
    Plug_pf_manager&                        plug_pf_manager;
    std::map<std::string,int>               cmds;

};

}



#endif
